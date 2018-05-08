#include "config_module.h"
#include RTAPI_INC_SLAB_H
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_math.h"
#include "hal.h"
#include "hostmot2.h"


#define MaxTrMessages     (16) // Send counts are written to 16 deep FIFO, burst mode

// Tx mode register errors
#define  TxSCFIFOError  (214)  // Tx Send Count FIFO Error


//Rx mode register errors
#define RxRCFIFOError   (114)  // RCFIFO Error, Bit  4  
//#define RxOverrunError  (111)  // Overrun error (no stop bit when expected) (sticky), Bit  1
//#define RxStartbitError (110)  // False Start bit error (sticky), Bit  0

// Rx count register errors
//#define RxPacketOverrrunError (1115)    // Bit 15         Overrun error in this packet
//#define RxPacketStartbitError (1114)    // Bit 14         False Start bit error in this packet

// the next two error conditions
// are very unprobable, but we consider them nevertheless
#define RxPacketSizeZero      (1120)    // the length of the received packet is 0
#define RxArraySizeError      (1140)    // sizeof(data array)= num_frames*max_frame_length is too small for all the data in the buffer

int hm2_mechatrolink_parse_md(hostmot2_t *hm2, int md_index)
{
    // All this function actually does is allocate memory
    // and give the mechatrolink modules names.


    //
    // some standard sanity checks
    //

    int i, r = -EINVAL;
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    static int last_gtag = -1;

    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 11, 4, 0x0007)) {
        HM2_ERR("inconsistent Module Descriptor!\n");
        return -EINVAL;
    }

    if (hm2->mechatrolink.num_instances > 1 && last_gtag == md->gtag) {
        HM2_ERR(
                "found duplicate Module Descriptor for %s (inconsistent "
                "firmware), not loading driver %i %i\n",
                hm2_get_general_function_name(md->gtag), md->gtag, last_gtag
                );
        return -EINVAL;
    }
    last_gtag = md->gtag;

    if (hm2->config.num_mechatrolinks > md->instances) {
        HM2_ERR(
                "config defines %d mechatrolinks, but only %d are available, "
                "not loading driver\n",
                hm2->config.num_mechatrolinks,
                md->instances
                );
        return -EINVAL;
    }

    if (hm2->config.num_mechatrolinks == 0) {
        return 0; 
    }

    //
    // looks good, start, or continue, initializing
    //

    if (hm2->mechatrolink.num_instances == 0){
        if (hm2->config.num_mechatrolinks == -1) {
            hm2->mechatrolink.num_instances = md->instances;
        } else {
            hm2->mechatrolink.num_instances = hm2->config.num_mechatrolinks;
        }

        hm2->mechatrolink.instance = (hm2_mechatrolink_instance_t *)hal_malloc(hm2->mechatrolink.num_instances
                                                               * sizeof(hm2_mechatrolink_instance_t));
        if (hm2->mechatrolink.instance == NULL) {
            HM2_ERR("out of memory!\n");
            r = -ENOMEM;
            return r;
        }
    }

    for (i = 0 ; i < hm2->mechatrolink.num_instances ; i++){
        hm2_mechatrolink_instance_t *inst = &hm2->mechatrolink.instance[i];
        if (md->gtag == HM2_GTAG_MECHATROLINK) {
            inst->tx_addr = md->base_address + i * md->instance_stride;
            inst->tx_fifo_station_addr = (md->base_address
                                          + md->register_stride
                                          + i * md->instance_stride);
            inst->tx_fifo_count_addr = (md->base_address
                                        + 2 * md->register_stride
                                        + i * md->instance_stride);
            inst->rx_addr = (md->base_address
                             + 3 * md->register_stride
                             + i * md->instance_stride);
            inst->rx_fifo_station_addr = (md->base_address
                                          + 4 * md->register_stride
                                          + i * md->instance_stride);
            inst->rx_fifo_count_addr = (md->base_address
                                        + 5 * md->register_stride
                                        + i * md->instance_stride);
            inst->mode_addr = (md->base_address
                               + 6 * md->register_stride
                               + i * md->instance_stride);
            inst->user_par_reg_addr0 = (md->base_address
                                        + 7 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg_addr1 = (md->base_address
                                        + 8 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg_addr2 = (md->base_address
                                        + 9 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg_addr3 = (md->base_address
                                        + 10 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg_addr4 = (md->base_address
                                        + 11 * md->register_stride
                                        + i * md->instance_stride);
        }
        else {
            HM2_ERR("Something very wierd happened");
            return r;
        }

    }

    return hm2->mechatrolink.num_instances;
}

EXPORT_SYMBOL_GPL(hm2_mechatrolink_setup);
int hm2_mechatrolink_setup(char *name, hm2_mechatronlik_user_par_t user_par){
    hostmot2_t *hm2;
    hm2_mechatrolink_instance_t *inst = 0;
    u32 buff;
    int i,r;

    i = hm2_get_mechatrolink(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find mechatrolink instance %s.\n", name);
        return -EINVAL;
    }
    inst = &hm2->mechatrolink.instance[i];

    r = 0;
    /* write user parameters, 5 registers in total */
    buff = (u32)user_par.mod & ((u32)user_par.ma << 16u);
    r += hm2->llio->write(hm2->llio, inst->user_par_reg_addr0, &buff, sizeof(u32));
    buff = (u32)user_par.ma_max & ((u32)user_par.t_mcyc << 16u);
    r += hm2->llio->write(hm2->llio, inst->user_par_reg_addr1, &buff, sizeof(u32));
    buff = (u32)user_par.t_cyc & ((u32)user_par.byte << 16u);
    r += hm2->llio->write(hm2->llio, inst->user_par_reg_addr2, &buff, sizeof(u32));
    buff = (u32)user_par.dev & ((u32)user_par.max_rtry << 16u);
    r += hm2->llio->write(hm2->llio, inst->user_par_reg_addr3, &buff, sizeof(u32));
    buff = (u32)user_par.c2m_ch & ((u32)user_par.wdt << 16u);
    r += hm2->llio->write(hm2->llio, inst->user_par_reg_addr4, &buff, sizeof(u32));

    /* start mechatrolink master instance */

    if (r < 0) {
        HM2_ERR("Mechatrolink: hm2->llio->write failure %s\n", name);
        return -1;
    }

    return 0;
}

EXPORT_SYMBOL_GPL(hm2_mechatrolink_send);
int hm2_mechatrolink_send(char *name,  unsigned char data[], u8 *num_messages, u8 message_sizes[], u8 message_stations[])
{
    hostmot2_t *hm2;
    u32 buff;
    int r, c, i;
    int inst;

    inst = hm2_get_mechatrolink(&hm2, name);
    if (inst < 0) {
        HM2_ERR_NO_LL("Can not find Mechatrolink instance %s.\n", name);
        return -EINVAL;
    }
    /* if (hm2->mechatrolink.instance[inst].bitrate == 0){ */
    /*     HM2_ERR("%s has not been configured.\n", name); */
    /*     return -EINVAL; */
    /* } */

    c = 0;
    u8 count = 0;
    /*
       we work with nmessages as a local copy of num_frames,
       so that we can return the num_frames sent out
       in case of SCFIFO error.
     */
    u8 nmessages = *num_messages;

    /* http://freeby.mesanet.com/regmap
       Send counts are written to 16 deep FIFO allowing up to 16 packets to be
       sent in a burst (subject to data FIFO depth limits).
    */
    // Test if num_messages <= MaxTrMessages
    if ((*num_messages) > MaxTrMessages){
        nmessages = MaxTrMessages;
    } else {
        nmessages = *num_messages;
    }

    *num_messages = 0;

    for (i = 0; i < nmessages; i++) {
        count = count + message_sizes[i];
        while (c < count - 3){
               buff = (data[c] +
                      (data[c + 1] << 8) +
                      (data[c + 2] << 16) +
                      (data[c + 3] << 24));
               r = hm2->llio->write(hm2->llio, hm2->mechatrolink.instance[inst].tx_addr,
                                    &buff, sizeof(u32));
               if (r < 0) {
                   HM2_ERR("%s send: hm2->llio->write failure\n", name);
                   return -1;
               }
              c = c + 4;
        }

        // Now write the last bytes with bytes number < 4
        switch(count - c) {
        case 0:
            break;
        case 1:
            buff = data[c];
            r = hm2->llio->write(hm2->llio, hm2->mechatrolink.instance[inst].tx_addr,
                                 &buff, sizeof(u32));
            if (r < 0){
                HM2_ERR("%s send: hm2->llio->write failure\n", name);
                return -1;
            }
            break;
        case 2:
            buff = (data[c] +
                    (data[c+1] << 8));
            r = hm2->llio->write(hm2->llio, hm2->mechatrolink.instance[inst].tx_addr,
                                 &buff, sizeof(u32));
            if (r < 0){
                HM2_ERR("%s send: hm2->llio->write failure\n", name);
                return -1;
            }
            break;
        case 3:
            buff = (data[c] +
                    (data[c+1] << 8) +
                    (data[c+2] << 16));
            r = hm2->llio->write(hm2->llio, hm2->mechatrolink.instance[inst].tx_addr,
                                 &buff, sizeof(u32));
            if (r < 0){
                HM2_ERR("%s send: hm2->llio->write failure\n", name);
                return -1;
            }
            break;
        default:
            HM2_ERR("%s send error in buffer parsing: count = %i, i = %i\n", name, count, c);
            return -1;
        } // end switch

        // Write the number of bytes to be sent to Mechatrolink sendcount register
        buff = (u32) message_sizes[i];
        r = hm2->llio->write(hm2->llio, hm2->mechatrolink.instance[inst].tx_fifo_count_addr,
                             &buff, sizeof(u32));
        // Write the station address this message should be sent to
        buff = (u32) message_stations[i];
        r = hm2->llio->write(hm2->llio, hm2->mechatrolink.instance[inst].tx_fifo_station_addr,
                             &buff, sizeof(u32));
        // Check for Send Count FIFO error
        r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].mode_addr,
                            &buff, sizeof(u32));

        if ((buff >> 4) & 0x01) {
            HM2_ERR_NO_LL("%s: SCFFIFO error\n", name);
            return -TxSCFIFOError;
        }

        if (r < 0) {
            HM2_ERR("%s send: hm2->llio->write failure\n", name);
            return -1;
        }

        (*num_messages)++;
        c = count;
    } // send message loop

    return count;
}

EXPORT_SYMBOL_GPL(hm2_mechatrolink_read);
int hm2_mechatrolink_read(char *name, unsigned char data[], u8 *num_messages, u8 *max_message_length, u8 message_sizes[], u8 message_stations[])
{
    hostmot2_t *hm2;
    int i, r, c;
    int bytes_total = 0; // total amount of bytes read
    u8 message_count; // packets count
    u8 byte_count; // bytes count for the oldest packet received
    u8 station_address;
    int inst;
    u32 buff;
    u16 data_size = (*num_messages)*(*max_message_length);

    inst = hm2_get_mechatrolink(&hm2, name);

    if (inst < 0) {
        HM2_ERR_NO_LL("Can not find Mechatrolink instance %s.\n", name);
        *num_messages=0;
        return -EINVAL;
    }
    /* if (hm2->mechatrolink.instance[inst].mode == 0) { */
    /*     HM2_ERR("%s has not been configured.\n", name); */
    /*     *num_messages=0; */
    /*     return -EINVAL; */
    /* } */

    // First poll the mode register for a non zero frames recieved count 
    // (mode register bits 20..16)
    r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].mode_addr,
                        &buff, sizeof(u32));
    if (r < 0) {
        HM2_ERR("%s read: hm2->llio->read failure\n", name);
                return -1; // make the error message more detailed
    }
    message_count = (buff >> 16)  & 0x1f;
    // We expect to read at least 1 message.
    // If there is no complete message yet in the buffer,
    // we'll deal with this by checking error bits.
    *num_messages = 0;

    // TODO: check errors

    if (message_count == 0) {
        HM2_ERR_NO_LL("%s: no new messages \n", name);
        return 0;       // return zero bytes
    }

    for (i = 0; i < message_count; i++) {
          buff=0;
          /* The receive count register is a FIFO that contains the byte counts
             of recieved packets. Since it is a FIFO it must only be read once after it
             has be determined that there are packets available to read. */
          r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].rx_fifo_count_addr,
                              &buff, sizeof(u32));

          if (r < 0) {
              HM2_ERR("%s read: hm2->llio->read failure\n", name);
              return -1; // make the error message more detailed
          }

          byte_count = buff & 0xFF; // Mechatrolink  receive count register Bits 8..0 : bytes in receive packet

          // TODO: check error bits 9..11 of rx_fifo_count register

          r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].rx_station_addr,
                              &buff, sizeof(u32));

          if (r < 0) {
              HM2_ERR("%s read: hm2->llio->read failure\n", name);
              return -1; // make the error message more detailed
          }

          station_address = buff & Ox1F; // Mechatrolink RX station register 5 bits station address

          // a packet is completely received, but its byte count is zero
          // is very unprobable, however we intercept this error too
          if (byte_count==0) {
              HM2_ERR_NO_LL("%s: packet %d has %d bytes.\n", name, message_count+1, byte_count);
              return -RxPacketSizeZero;
          }

          if ((bytes_total + byte_count) > data_size) {
              HM2_ERR_NO_LL("%s: bytes avalaible %d are more than data array size %d\n", name, bytes_total + byte_count, data_size);
              return -RxArraySizeError ;
          }

          (*num_messages)++; // increment num_frames to be returned at the end
          c = 0;
          buff = 0;
          message_sizes[i] = byte_count;
          message_stations[i] = station_address;

          while (c < byte_count - 3){
              r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].rx_addr,
                                  &buff, sizeof(u32));

              if (r < 0) {
                  HM2_ERR("%s read: hm2->llio->read failure\n", name);
                  return r;
              }

              data[bytes_total + c] = (buff & 0x000000FF); // i*frame_sizes[i]
              data[bytes_total + c + 1] = (buff & 0x0000FF00) >> 8;
              data[bytes_total + c + 2] = (buff & 0x00FF0000) >> 16;
              data[bytes_total + c + 3] = (buff & 0xFF000000) >> 24;
              c = c + 4;
          }

          switch (byte_count - c) {
                 case 0:
                      break;
                 case 1:
                      r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].rx_addr,
                                          &buff, sizeof(u32));
                      data[bytes_total + c]   = (buff & 0x000000FF);
                      break;
                 case 2:
                      r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].rx_addr,
                                          &buff, sizeof(u32));
                      data[bytes_total + c]   = (buff & 0x000000FF);
                      data[bytes_total + c + 1] = (buff & 0x0000FF00) >> 8;
                      break;
                 case 3:
                      r = hm2->llio->read(hm2->llio, hm2->mechatrolink.instance[inst].rx_addr,
                                          &buff, sizeof(u32));
                      data[bytes_total + c]   = (buff & 0x000000FF);
                      data[bytes_total + c + 1] = (buff & 0x0000FF00) >> 8;
                      data[bytes_total + c + 2] = (buff & 0x00FF0000) >> 16;
                      break;
                default:
                     HM2_ERR_NO_LL("Mechatrolink READ: Error in buffer parsing.\n");
                     return -EINVAL;
          }

          bytes_total = bytes_total + byte_count;
    } // message loop

    return bytes_total;
}

void hm2_mechatrolink_print_module(hostmot2_t *hm2){
    int i;
    HM2_PRINT("Mechatrolink: %d\n", hm2->mechatrolink.num_instances);
    if (hm2->mechatrolink.num_instances <= 0) return;
    HM2_PRINT("    version: %d\n", hm2->mechatrolink.version);
    HM2_PRINT("    master configurations\n");
    for (i = 0; i < hm2->mechatrolink.num_instances; i ++) {
        /* HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n",  */
        /*           hm2->mechatrolink.instance[i].clock_freq,  */
        /*           hm2_hz_to_mhz(hm2->mechatrolink.instance[i].clock_freq)); */
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("    HAL name = %s\n", hm2->mechatrolink.instance[i].name);
    }
}

// The following standard Hostmot2 functions are not currently used by mechatrolink. 

void hm2_mechatrolink_cleanup(hostmot2_t *hm2)
{
}

void hm2_mechatrolink_write(hostmot2_t *hm2)
{
}
