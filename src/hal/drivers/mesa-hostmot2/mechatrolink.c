#include "config_module.h"
#include RTAPI_INC_SLAB_H
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_math.h"
#include "hal.h"
#include "hostmot2.h"


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

    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 10, 4, 0x0007)) {
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
            inst->tx_fifo_count_addr = (md->base_address
                                        + md->register_stride
                                        + i * md->instance_stride);
            inst->rx_addr = (md->base_address
                             + 2 * md->register_stride
                             + i * md->instance_stride);
            inst->rx_fifo_count_addr = (md->base_address
                                        + 3 * md->register_stride
                                        + i * md->instance_stride);
            inst->control_addr = (md->base_address
                                  + 4 * md->register_stride
                                  + i * md->instance_stride);
            inst->user_par_reg0_addr = (md->base_address
                                        + 5 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg1_addr = (md->base_address
                                        + 6 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg2_addr = (md->base_address
                                        + 7 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg3_addr = (md->base_address
                                        + 8 * md->register_stride
                                        + i * md->instance_stride);
            inst->user_par_reg4_addr = (md->base_address
                                        + 9 * md->register_stride
                                        + i * md->instance_stride);
        }
        else {
            HM2_ERR("Something very wierd happened");
            return r;
        }

    }

    return hm2->mechatrolink.num_instances;
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

EXPORT_SYMBOL_GPL(hm2_mechatrolink_setup);
int hm2_mechatrolink_setup(char *name){
    hostmot2_t *hm2;
    hm2_pktuart_instance_t *inst = 0;
    u32 buff;
    int i,r;

    i = hm2_get_pktuart(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find mechatrolink instance %s.\n", name);
        return -EINVAL;
    }
    inst = &hm2->pktuart.instance[i];

    // TODO: add setup code

    if (r < 0) {
        HM2_ERR("Mechatrolink: hm2->llio->write failure %s\n", name);
        return -1;
    }
}
