/*
 HAL driver to talk to P260C I/O boards on an RS-485 chain.




*/

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */

#include "hal.h"		/* HAL public API decls */

#include "mesa-hostmot2/hostmot2.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif

#define MODNAME "mesa_p260c"

/* module information */
MODULE_AUTHOR("Alexander Rössler");
MODULE_DESCRIPTION("Driver for P260C boards in a RS-485 string via the MESA UART interface. Version 1.0");
MODULE_LICENSE("GPL");

#define MAX_BOARDS 16u
#define INPUT_PINS 16u
#define OUTPUT_PINS 16u
#define MAX_FRAME_SIZE 4u
#define FRAME_SIZE 4u
#define BAUDRATE 3000000
//#define BAUDRATE 115200
#define CLOCK_LOW_MHZ  0.100

//#define DEBUG_RX 1

typedef struct _board_s {
    // Board physical address setting
    u8        address;

    // HAL Pins
    hal_bit_t *input_pins[INPUT_PINS];
    hal_bit_t *output_pins[INPUT_PINS];
    // Status data
    s32        invalid_timer;             // count since last error
    s32        count_errors;              // count of errors per time if >X then comm_error
    hal_s32_t *invalidcnt;                // s32 total count of invalid reads
    hal_bit_t *comm_error;                // Currently in communication error
    hal_bit_t *permanent_error;           // Triggered permanent error ( must be reset )
    // Debug data
#ifdef DEBUG_RX
    hal_s32_t *writecnt;                  // s32 count of write calls
    hal_s32_t *readbytes;                 // s32 count of read bytes
    hal_s32_t *validcnt;                  // s32 count of valid read
    hal_s32_t *readbeforewritecount;
    hal_s32_t *readcount;
    hal_s32_t *read0;
    hal_s32_t *read1;
    hal_s32_t *read2;
    hal_s32_t *read3;
    hal_s32_t *maxreadtime;
#endif

    // Protocol data structures
    u16  output_bits;
    u8   output_data[4];

    u16  input_bits;
    u8   input_data[16];
    s32  input_cnt;
    bool input_valid;

} board_t;

static const char *modname = MODNAME;

static char *addrs;
RTAPI_MP_STRING( addrs, "board addresses, comma separated.  0-F");
static char *uart_channel_name;
RTAPI_MP_STRING( uart_channel_name, "UART channel name");

static int comp_id;

unsigned long runtime;
unsigned long threadtime;

typedef struct _mod_status_s {
    hal_s32_t *maxruntime;
    hal_s32_t *minruntime;
    hal_s32_t *maxreadtime;
    hal_s32_t *maxwritetime;

    hal_s32_t *writecnt;                  // s32 count of write calls

    hal_bit_t *comm_error;               // Currently some board has a communication error
    hal_bit_t *permanent_error;          // Permanent error triggered by comm_error ( Must be reset )
    hal_bit_t *reset_permanent;          // Input bit to reset permanent error

    // Parameters
    hal_s32_t *clear_comm_count;
    hal_s32_t *set_perm_count;
    hal_s32_t *min_tx_boards;
    hal_s32_t *max_rx_wait;

    hal_bit_t *debug_on_error;
} mod_status_t;

//#define ERROR_CLEAR_TIME     10  // 10 cycles without an error to clear the comm_error
//#define MAX_ERRORS_PER_TIME  5   // 5 errors without 10 clean, count as a comm_error

mod_status_t *mstat;

static int num_boards = 0;
board_t *boards;

typedef struct _write_buffer_s {
    u8 data[MAX_BOARDS][MAX_FRAME_SIZE];
    u16 size[MAX_BOARDS];
    u8 count;
} write_buffer_t;

static write_buffer_t write_buffer;

static void serial_port_task( void *arg, long period );

static int openserial(char *devicename, int baud );

int init_hal_module()
{
    comp_id = hal_init(modname);
    if(comp_id < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
        return -1;
    }
    return 0;
}

int allocate_board_structures()
{
    boards = hal_malloc( MAX_BOARDS * sizeof( board_t ) );
    if ( boards == NULL )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
        return -1;
    }
    memset( boards, 0, MAX_BOARDS * sizeof( board_t ) );
    return 0;
}

int allocate_status_structure()
{
    mstat = hal_malloc( sizeof( mod_status_t ) );
    if ( mstat == NULL )
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
        return -1;
    }
    memset( mstat, 0, sizeof( mod_status_t ) );
    return 0;
}

int parse_parameters()
{
    char *data;
    char *token;

    if (uart_channel_name == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: must define a pktUART name", modname);
        return -1;
    }

    if ( addrs != NULL )
    {
        data = addrs;
        while((token = strtok(data, ",")) != NULL)
        {
            int add = strtol(token, NULL, 16);

            if ( add < 0 || add > 15 )
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: address %s = %x is not valid. Only 0-F\n", modname, token, add );
                return -1;
            }
            boards[num_boards++].address = add;

            data = NULL;
        }
    }
    else
    {
        // No parameteres default to 1 boards address 0
        boards[0].address = 0;
        num_boards = 1;
    }
    return 0;
}

int open_and_configure_serial_port()
{
    return openserial(uart_channel_name, BAUDRATE);
}

int close_serial_port()
{
    return 0;
}

int export_pins()
{
    int i, j, retval;

    for (i = 0; i < num_boards; i++)
    {
        int add = boards[i].address;
        for (j = 0; j < INPUT_PINS; j++)
        {
            retval = hal_pin_bit_newf(HAL_OUT, &(boards[i].input_pins[j]), comp_id, "%s.%d.pin-%02d-in", modname, add, j+1);
            if(retval < 0)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin %d.%02d could not export pin, err: %d\n", modname, add, j+1, retval);
                return -1;
            }

        }
        for (j = 0; j < OUTPUT_PINS; j++)
        {
            retval = hal_pin_bit_newf(HAL_IN, &(boards[i].output_pins[j]), comp_id, "%s.%d.pin-%02d-out", modname, add, j+1);
            if(retval < 0)
            {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin %d.%02d could not export pin, err: %d\n", modname, add, j+1, retval);
                return -1;
            }

        }

        retval = hal_pin_s32_newf(HAL_IN, &(boards[i].invalidcnt), comp_id, "%s.%d.rx_cnt_error", modname, add );
        if(retval < 0)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin %d.serial_invalidcnt could not export pin, err: %d\n", modname, add, retval);
            return -1;
        }
        retval = hal_pin_bit_newf(HAL_OUT, &(boards[i].comm_error), comp_id, "%s.%d.rx_comm_error", modname, add );
        if(retval < 0)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin %d.comm_error could not export pin, err: %d\n", modname, add, retval);
            return -1;
        }
        retval = hal_pin_bit_newf(HAL_OUT, &(boards[i].permanent_error), comp_id, "%s.%d.rx_perm_error", modname, add );
        if(retval < 0)
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin %d.permanent_error could not export pin, err: %d\n", modname, add, retval);
            return -1;
        }

        // Debug
#ifdef DEBUG_RX
        hal_pin_s32_newf(HAL_IN, &(boards[i].writecnt), comp_id, "%s.%d.serial_writecnt", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].readbytes), comp_id, "%s.%d.serial_readbytes", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].validcnt), comp_id, "%s.%d.serial_validcnt", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].readbeforewritecount), comp_id, "%s.%d.serial_readbeforewrcnt", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].readcount), comp_id, "%s.%d.serial_readcnt", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].read0), comp_id, "%s.%d.serial_read0", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].read1), comp_id, "%s.%d.serial_read1", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].read2), comp_id, "%s.%d.serial_read2", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].read3), comp_id, "%s.%d.serial_read3", modname, add );
        hal_pin_s32_newf(HAL_IN, &(boards[i].maxreadtime), comp_id, "%s.%d.serial_maxreadtime", modname, add );
#endif
    }
    retval = hal_pin_s32_newf(HAL_IN, &(mstat->maxreadtime), comp_id, "%s.sys_max_read", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin maxreadtime could not export pin, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_s32_newf(HAL_IN, &(mstat->maxwritetime), comp_id, "%s.sys_max_write", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin maxwritetime could not export pin, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_s32_newf(HAL_IN, &(mstat->writecnt), comp_id, "%s.sys_writecnt", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin writecnt could not export pin, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_bit_newf(HAL_OUT, &(mstat->comm_error), comp_id, "%s.rx_comm_error", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin comm_error could not export pin, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_bit_newf(HAL_OUT, &(mstat->permanent_error), comp_id, "%s.rx_perm_error", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin permanent_error could not export pin, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(mstat->reset_permanent), comp_id, "%s.rx_reset_error", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin reset_permanent could not export pin, err: %d\n", modname, retval);
        return -1;
    }

    // Parameters
    retval = hal_pin_s32_newf(HAL_IO, &(mstat->clear_comm_count), comp_id, "%s.clear_comm_count", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: param clear_comm_count could not create, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_s32_newf(HAL_IO, &(mstat->set_perm_count), comp_id, "%s.set_perm_count", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: param set_perm_count could not create, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_s32_newf(HAL_IO, &(mstat->min_tx_boards), comp_id, "%s.minimum_tx", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: param minimum_tx could not create, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_s32_newf(HAL_IO, &(mstat->max_rx_wait), comp_id, "%s.max_rx_wait", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: param minimum_tx could not create, err: %d\n", modname, retval);
        return -1;
    }
    retval = hal_pin_bit_newf(HAL_IO, &(mstat->debug_on_error), comp_id, "%s.debug_on_error", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: param debug_on_error could not create, err: %d\n", modname, retval);
        return -1;
    }

    return 0;
}

void init_module_status()
{
    *(mstat->set_perm_count) = 5;
    *(mstat->clear_comm_count) = 10;
    *(mstat->min_tx_boards) = 6;
    *(mstat->max_rx_wait) = 5000000;
}

void init_write_buffer()
{
    int i;
    write_buffer.count = 0;
    for (i = 0; i < MAX_BOARDS; ++i) {
        write_buffer.size[i] = FRAME_SIZE;
    }
}

int export_functions()
{
    int retval;
    char  name[HAL_NAME_LEN + 1];

    rtapi_snprintf( name, sizeof(name), "%s.refresh", modname );
    retval = hal_export_funct( name, serial_port_task, 0, 0, 0, comp_id);
    if(retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: refresh funct export failed\n", modname);
        return -1;
    }
    return 0;
}

int rtapi_app_main(void)
{
    if (init_hal_module() != 0) {
        goto error;
    }
    if (allocate_board_structures() != 0) {
        goto error;
    }
    if (allocate_status_structure() != 0) {
        goto error;
    }

    if (parse_parameters() != 0) {
        goto error;
    }

    if (open_and_configure_serial_port() != 0) {
        goto error;
    }

    if (export_pins() != 0) {
        goto error;
    }
    if (export_functions() != 0) {
        goto error;
    }
    init_module_status();
    init_write_buffer();

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
    hal_ready(comp_id);

    return 0;

error:
    hal_exit(comp_id);
    return -1;
}

void rtapi_app_exit(void)
{
    close_serial_port();
    hal_exit(comp_id);
}

/***********************************************************************
Implementation
*/
static u32 calculate_filter_reg(int baud)
{
    double clock_low = CLOCK_LOW_MHZ; //in MHz, from IDROMConst.vhd
    double bit_time = 1e9 / (double)baud; // bit time in ns
    return (u32)(0.5 * bit_time * clock_low) - 1u;
}

/* http://freeby.mesanet.com/regmap */
#define DRIVE_ENABLE_BIT (1u << 6)
#define DRIVE_ENABLE_AUTO (1u << 5)
#define RX_ENABLE (1u << 3)
#define RX_MASK_ENABLE (1u << 2)
#define CLEAR_RX 1
#define CLEAR_TX 1
static int openserial(char *devicename, int baud)
{
    int retval;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Set up PktUART now\n", modname);

    u32 rx_filter_reg = calculate_filter_reg(baud);
    u32 tx_inter_frame_delay = 1u; // 226 bit = 76us
    u32 rx_inter_frame_delay = 113u; // 113 bit = 38us
    u8  tx_drive_enable_delay = 0u; // 15 clock low periods = 10ns
    u32 tx_data = (tx_inter_frame_delay << 8) | DRIVE_ENABLE_AUTO | (tx_drive_enable_delay & 0xF);
    u32 rx_data = (rx_filter_reg << 22) | (rx_inter_frame_delay << 8) | RX_ENABLE | RX_MASK_ENABLE;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: TX inter-frame delay: %u, RX inter-frame delay: %u, RX FilterReg: %u\n",
                    modname, tx_inter_frame_delay, rx_inter_frame_delay, rx_filter_reg);

    retval = hm2_pktuart_setup(devicename, baud, tx_data, rx_data, CLEAR_TX, CLEAR_RX);
    if (retval != 0){
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: PktUART for P260C setup problem: %d\n", modname, retval);
        return -1;
    }

    return 0;
}

/***********************************************************************

 Protocol task functions

***/

static int write_all_data()
{
    int retval;

    if (write_buffer.count == 0u) {
        return 0;
    }
    retval = hm2_pktuart_send(uart_channel_name, (u8*)(write_buffer.data), &(write_buffer.count), write_buffer.size);

    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: sending data failed\n", modname);
    }
    else {
        rtapi_print_msg(RTAPI_MSG_INFO, "%s sent: bytes %d, frames %u\n", uart_channel_name, retval, write_buffer.count);
    }
    write_buffer.count = 0u;

    return retval;
}

static void enqueue_write_data(u8 *data, u16 size)
{
    u16 i;
    for (i = 0; i < size; ++i) {
        write_buffer.data[write_buffer.count][i] = data[i];
    }
    write_buffer.size[write_buffer.count] = size;
    write_buffer.count++;
}

static void enqueue_board_write_data( int board )
{
    enqueue_write_data(boards[board].output_data, FRAME_SIZE);

#ifdef DEBUG_RX
    if ( boards[board].writecnt != NULL )
    {
        *(boards[board].writecnt) = *(boards[board].writecnt) + 1;
    }
#endif

}

static unsigned char nibble_xsum( unsigned char *data )
{
    unsigned char xsum;
    int i;

    xsum = 0;
    for ( i=0; i<4; i++ )
    {
        xsum = xsum ^ (data[i]>>4);
        xsum = xsum ^ (data[i] & 0xf );
    }

    return xsum;
}

static u8 validate_input_buffer( u8 *input_data, u16 *bits )
{
    u8 input_address = input_data[0] >> 3;

    if ( nibble_xsum( input_data ) == 0 )
    {
        if ( (input_data[0] & 0x7) == 2 )
        {
            if ( (input_data[3] & 0xF) == 3 )
            {
                *bits = (input_data[1]<<8) & 0xFF00;
                *bits |= (input_data[2] & 0xFF);

                return input_address;
            }
        }
    }
    return -1;
}

//
// Count RX valids.
//   gets called once per cycle for each board.
//
static int read_counts( int board )
{
    int ret = 0;

    if ( boards[board].input_valid )
    {
        // Good data
        if ( boards[board].invalid_timer >= *(mstat->clear_comm_count) )
        {
            // Max out
            boards[board].invalid_timer = *(mstat->clear_comm_count);
            // Clear error counter
            boards[board].count_errors = 0;
            // Clear comm error
            if ( boards[board].comm_error != NULL )
            {
                *(boards[board].comm_error) = 0;
            }
        }
#ifdef DEBUG_RX
        if ( boards[board].validcnt != NULL )
        {
            *(boards[board].validcnt) = *(boards[board].validcnt) + 1;
        }
#endif
    }
    else
    {
        ret=1;

        // new error
        boards[board].count_errors++;
        // reset timer
        boards[board].invalid_timer = 0;
        // Total Count
        if ( boards[board].invalidcnt != NULL )
        {
            *(boards[board].invalidcnt) = *(boards[board].invalidcnt) + 1;
        }
    }
    if ( boards[board].count_errors >= *(mstat->set_perm_count) )
    {
        // Max out.
        boards[board].count_errors = *(mstat->set_perm_count);
        // Set comm error
        if ( boards[board].comm_error != NULL )
        {
            *(boards[board].comm_error) = 1;
        }
        // Go into permenant error
        if ( boards[board].permanent_error != NULL )
        {
            *(boards[board].permanent_error) = 1;
        }
    }

    boards[board].invalid_timer++;

    return ret;
}
//
// Read for a maximum of 8 bytes time and timeout
// 8 bytes at 3,000,000 baud is about 30 usec
// wait for 1/2 msec since the usb turn around is long.
//

unsigned long last_readtime;

static void read_all_data()
{
    int i, j;
    int valid = 0;
    u8 input_address;
    u16 bits;
    u16 max_frame_length = MAX_FRAME_SIZE;
    u8 num_frames = MAX_BOARDS;
    u8 reply_data[num_frames * max_frame_length];
    u16 frame_sizes[MAX_BOARDS];
    int rxbytes;
    u16 data_pos = 0u;

#ifdef DEBUG_RX
    if ( boards[0].readcount != NULL )
    {
        *(boards[0].readcount) = 0;
    }
#endif

    rxbytes = hm2_pktuart_read(uart_channel_name, reply_data, &num_frames, &max_frame_length, frame_sizes);
    rtapi_print_msg(RTAPI_MSG_INFO, "PktUART receive: got %d bytes, %d frames\n", rxbytes, num_frames);

    for (i = 0; i < num_frames; ++i)
    {
        rtapi_print_msg(RTAPI_MSG_INFO, "Frame size %i: %d", i, frame_sizes[i]);

        if ((input_address = validate_input_buffer(&reply_data[data_pos], &bits)) >= 0)
        {
            // Find the board for this data
            for (j = 0; j < num_boards; j++)
            {
                if ( boards[j].address == input_address )
                {
                    boards[j].input_bits  = bits;
                    boards[j].input_valid = true;
                    boards[j].input_cnt = frame_sizes[i];
                    valid++;
                    break;
                }
            }
            // Check for no boards...
            if ( j == num_boards)
            {
                // error, valid data for a board we don't have.
            }
        }

        data_pos += frame_sizes[i];
    }

#if DEBUG_RX
    // Count all bytes in the first board
    if ( boards[0].readbytes != NULL )
    {
        *(boards[0].readbytes) += data_pos;
    }
    if ( boards[0].readcount != NULL )
    {
        *(boards[0].readcount) += data_pos;
    }
#endif

    valid = 0;
    for (i = 0; i < num_boards; i++)
    {
        valid += read_counts(i);
    }
}

// get HAL pins and set data
static void set_output( int board )
{
    int i, bit;

    // Get bits from HAL pins
    bit = 1;
    for (i = 0; i < OUTPUT_PINS; i++)
    {
        if ( *(boards[board].output_pins[i]) )
        {
            boards[board].output_bits |= bit;
        }
        else
        {
            boards[board].output_bits &= ~bit;
        }

        bit = bit<<1;
    }

#if 0
    // If error, reset outputs
    if ( *mstat->permanent_error )
    {
        boards[board].output_bits = 0;
    }
#endif

    // Build protocol bytes
    boards[board].output_data[ 0 ] = (boards[board].address << 3);
    boards[board].output_data[ 0 ] |= 0x3;

    boards[board].output_data[ 1 ] = (boards[board].output_bits >> 8) & 0xFF;
    boards[board].output_data[ 2 ] = boards[board].output_bits & 0xFF;
    boards[board].output_data[ 3 ] = 0x3;
    boards[board].output_data[ 3 ] |= ( nibble_xsum( boards[board].output_data ) << 4 );
}

static void set_input( int board )
{
    int i, bit;

    // If error, clear inputs
    if ( *mstat->permanent_error )
    {
        boards[board].input_bits = 0;
    }

    bit = 1;
    for(i = 0; i < INPUT_PINS; i++)
    {
        if ( boards[board].input_bits & bit )
        {
            *(boards[board].input_pins[i]) = 1;
        }
        else
        {
            *(boards[board].input_pins[i]) = 0;
        }

        bit = bit << 1;
    }
}

static void handle_errors( void )
{
    int i, err;
    // Handle error reset
    if ( *mstat->reset_permanent )
    {
        *mstat->reset_permanent = 0;

        if ( ! *mstat->comm_error )
        {
            *mstat->permanent_error = 0;
            for ( i=0;i<num_boards;i++)
            {
                *(boards[i].permanent_error) = 0;
            }
        }
    }
    // Check for errors
    err = 0;
    for (i = 0; i < num_boards; i++)
    {
        if ( *(boards[i].comm_error) )
        {
            err = 1;
        }
    }
    *mstat->comm_error = err;
    if ( err )
    {
        *mstat->permanent_error = 1;
    }
}

static void serial_port_task( void *arg, long period )
{
    int i;
    u8 empty_frame[FRAME_SIZE] = { 0x00u, 0x00u, 0x00u, 0x00u };

#if TAKE_TIME
    unsigned long t0, t1;
    t0 = rtapi_get_time();

    // Update max time between thread calls.
    if ( threadtime && *(mstat->maxwritetime) < (t0-threadtime) )
    {
        *(mstat->maxwritetime) = (t0-threadtime);
    }

    // Check to be sure we are at the 10msec time < 9.5msec wait for the next tick.
    if ( (t0-threadtime) < 9500000 )
    {
        return;
    }
    threadtime = t0;
#endif

    *mstat->writecnt += 1;

#if 1

    // Start the transmit to the first board.
    for (i = 0; i < num_boards; i++)
    {
        // get pins from user
        set_output( i );

        enqueue_board_write_data( i );

        // Reset receive data flags
        boards[i].input_valid = false;
        boards[i].input_cnt = 0;
    }
    // Fill in minimum tx data with 0's
    for (i = num_boards; i < *(mstat->min_tx_boards); i++)
    {
        enqueue_write_data(empty_frame, FRAME_SIZE );
    }
#else
    u8 out_data[4];
    out_data[0] = 0b10101010;
    out_data[1] = 'a';
    out_data[2] = 'a';
    out_data[3] = 'a';
    enqueue_write_data(out_data, 1);
    //enqueue_write_data(out_data, 4);
#endif

    write_all_data();
    read_all_data();
    handle_errors();

    // Set pins to user
    for (i = 0; i < num_boards; i++)
    {
        set_input( i );
    }

#if TAKE_TIME
    t1 = rtapi_get_time();
    runtime = t1 - t0;
#endif
}

