#ifndef I2S_PARALLEL_H
#define I2S_PARALLEL_H

#include <stdint.h>
#include "soc/i2s_struct.h"

typedef enum {
    I2S_PARALLEL_BITS_8=8,
    I2S_PARALLEL_BITS_16=16,
    I2S_PARALLEL_BITS_32=32,
} i2s_parallel_cfg_bits_t;

/// @private
typedef struct {
    void *memory;
    size_t size;
} i2s_parallel_buffer_desc_t;

/// @private
typedef struct {
    int gpio_bus[24];
    int gpio_wclk;
    bool inv_wclk;
    int gpio_bclk;
    bool inv_bclk;
    int clkspeed;
    i2s_parallel_cfg_bits_t bits;
    i2s_parallel_buffer_desc_t *bufa;
    i2s_parallel_buffer_desc_t *bufb;
} i2s_parallel_config_t;

#ifdef __cplusplus
extern "C" {
#endif

int i2snum(i2s_dev_t *dev);
void i2s_parallel_setup(i2s_dev_t *dev, const i2s_parallel_config_t *cfg);
void i2s_parallel_flip_to_buffer(i2s_dev_t *dev, int bufid);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
