/*
 HAL driver for the mechatrolink bridge.

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
#include <stdbool.h>
#include <fcntl.h>
#include <errno.h>

#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif

#define MODNAME "mesa_mechatrolink"

/* module information */
MODULE_AUTHOR("Alexander Rössler");
MODULE_DESCRIPTION("Driver for the mechatrolink bridge interface for the MESA card. Version 1.0");
MODULE_LICENSE("GPL");

#define CLOCK_LOW_MHZ  0.100

static const char *modname = MODNAME;

static char *mechatrolink_master_name;
RTAPI_MP_STRING( mechatrolink_master_name, "Mechatrolink master name");

static int comp_id;

unsigned long runtime;
unsigned long threadtime;

typedef struct _mod_status_s {
    hal_s32_t *maxreadtime;
} mod_status_t;


mod_status_t *mstat;

static void communication_task( void *arg, long period );

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

int allocate_status_structure()
{
    mstat = hal_malloc(sizeof(mod_status_t));
    if (mstat == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
        return -1;
    }
    memset(mstat, 0, sizeof(mod_status_t));
    return 0;
}

int parse_parameters()
{
    /* char *data; */
    /* char *token; */

    if (mechatrolink_master_name == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: must define a Mechatrolink Master name", modname);
        return -1;
    }

    /* If (addrs != NULL) */
    /* { */
    /*     data = addrs; */
    /*     while((token = strtok(data, ",")) != NULL) */
    /*     { */
    /*         int add = strtol(token, NULL, 16); */

    /*         if (add < 0 || add > 15) */
    /*         { */
    /*             rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: address %s = %x is not valid. Only 0-F\n", modname, token, add ); */
    /*             return -1; */
    /*         } */
    /*         boards[num_boards++].address = add; */

    /*         data = NULL; */
    /*     } */
    /* } */
    /* else */
    /* { */
    /*     // No parameteres default to 1 boards address 0 */
    /*     boards[0].address = 0; */
    /*     num_boards = 1; */
    /* } */
    return 0;
}


int export_pins()
{
    int i, j, retval;

    retval = hal_pin_s32_newf(HAL_IN, &(mstat->maxreadtime), comp_id, "%s.sys_max_read", modname );
    if(retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin maxreadtime could not export pin, err: %d\n", modname, retval);
        return -1;
    }

    return 0;
}


void init_module_status()
{
    /* *(mstat->set_perm_count) = 5; */
    /* *(mstat->clear_comm_count) = 10; */
    /* *(mstat->min_tx_boards) = 1; */
    /* *(mstat->max_rx_wait) = 5000000; */
}

void init_write_buffer()
{
    /* int i; */
    /* write_buffer.count = 0; */
    /* for (i = 0; i < MAX_BOARDS; ++i) { */
    /*     write_buffer.size[i] = FRAME_SIZE; */
    /* } */
}

int export_functions()
{
    int retval;
    char  name[HAL_NAME_LEN + 1];

    rtapi_snprintf( name, sizeof(name), "%s.refresh", modname );
    retval = hal_export_funct( name, communication_task, 0, 0, 0, comp_id);
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
    /* if (allocate_board_structures() != 0) { */
    /*     goto error; */
    /* } */
    if (allocate_status_structure() != 0) {
        goto error;
    }

    if (parse_parameters() != 0) {
        goto error;
    }

    /* if (open_and_configure_serial_port() != 0) { */
    /*     goto error; */
    /* } */

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
    hal_exit(comp_id);
}

static void communication_task( void *arg, long period )
{
    
}
