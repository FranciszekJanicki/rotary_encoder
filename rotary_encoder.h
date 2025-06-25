#ifndef ROTARY_ENCODER_ROTARY_ENCODER_H
#define ROTARY_ENCODER_ROTARY_ENCODER_H

#include "rotary_encoder_config.h"

typedef struct {
    rotary_encoder_state_t state;
    rotary_encoder_config_t config;
    rotary_encoder_interface_t interface;
} rotary_encoder_t;

rotary_encoder_err_t rotary_encoder_initialize(rotary_encoder_t* encoder,
                                               rotary_encoder_config_t const* config,
                                               rotary_encoder_interface_t const* interface);
rotary_encoder_err_t rotary_encoder_deinitialize(rotary_encoder_t* encoder);

rotary_encoder_err_t rotary_encoder_reset(rotary_encoder_t* encoder);

rotary_encoder_err_t rotary_encoder_get_position(rotary_encoder_t* encoder, float32_t* position);
rotary_encoder_err_t rotary_encoder_get_speed(rotary_encoder_t* encoder,
                                              float32_t* speed,
                                              float32_t delta_time);
rotary_encoder_err_t rotary_encoder_get_acceleration(rotary_encoder_t* encoder,
                                                     float32_t* acceleration,
                                                     float32_t delta_time);

#endif // ROTARY_ENCODER_ROTARY_ENCODER_H