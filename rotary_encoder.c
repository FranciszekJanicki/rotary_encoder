#include "rotary_encoder.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static rotary_encoder_err_t rotary_encoder_device_init(rotary_encoder_t const* encoder)
{
    return encoder->interface.device_init
               ? encoder->interface.device_init(encoder->interface.device_user)
               : ROTARY_ENCODER_ERR_NULL;
}

static rotary_encoder_err_t rotary_encoder_device_deinit(rotary_encoder_t const* encoder)
{
    return encoder->interface.device_deinit
               ? encoder->interface.device_deinit(encoder->interface.device_user)
               : ROTARY_ENCODER_ERR_NULL;
}

static rotary_encoder_err_t rotary_encoder_device_get_step_count(rotary_encoder_t const* encoder,
                                                                 int64_t* step_count)
{
    return encoder->interface.device_get_step_count
               ? encoder->interface.device_get_step_count(encoder->interface.device_user,
                                                          step_count)
               : ROTARY_ENCODER_ERR_NULL;
}

static inline float32_t rotary_encoder_wrap_position(float32_t position)
{
    position = fmodf(position, 360.0F);

    while (position < 0.0F) {
        position += 360.0F;
    }
    if (position >= 360.0F) {
        position -= 360.0F;
    }

    return position;
}

static inline int64_t rotary_encoder_position_to_step_count(rotary_encoder_t const* encoder,
                                                            float32_t position)
{
    assert(encoder->config.step_change > 0.0F);

    float32_t step_count = rotary_encoder_wrap_position(position) / encoder->config.step_change;

    return (int64_t)roundf(step_count);
}

static inline float32_t rotary_encoder_step_count_to_position(rotary_encoder_t const* encoder,
                                                              int64_t step_count)
{
    assert(encoder->config.step_change > 0.0F);

    float32_t position = (float32_t)step_count * encoder->config.step_change;

    return rotary_encoder_wrap_position(position);
}

rotary_encoder_err_t rotary_encoder_initialize(rotary_encoder_t* encoder,
                                               rotary_encoder_config_t const* config,
                                               rotary_encoder_interface_t const* interface)
{
    assert(encoder && config && interface);

    memset(encoder, 0, sizeof(*encoder));
    memcpy(&encoder->config, config, sizeof(*config));
    memcpy(&encoder->interface, interface, sizeof(*interface));

    return rotary_encoder_device_init(encoder);
}

rotary_encoder_err_t rotary_encoder_deinitialize(rotary_encoder_t* encoder)
{
    assert(encoder);

    rotary_encoder_err_t err = rotary_encoder_device_deinit(encoder);

    memset(encoder, 0, sizeof(*encoder));

    return err;
}

rotary_encoder_err_t rotary_encoder_reset(rotary_encoder_t* encoder)
{
    assert(encoder);

    memset(&encoder->state, 0, sizeof(encoder->state));

    return ROTARY_ENCODER_ERR_OK;
}

rotary_encoder_err_t rotary_encoder_get_position(rotary_encoder_t* encoder, float32_t* position)
{
    assert(encoder && position);

    int64_t step_count;
    rotary_encoder_err_t err = rotary_encoder_device_get_step_count(encoder, &step_count);

    *position = rotary_encoder_step_count_to_position(encoder, step_count);

    return err;
}

rotary_encoder_err_t rotary_encoder_get_speed(rotary_encoder_t* encoder,
                                              float32_t* speed,
                                              float32_t delta_time)
{
    assert(encoder && speed && delta_time > 0.0F);

    float32_t position;
    rotary_encoder_err_t err = rotary_encoder_get_position(encoder, &position);

    *speed = (position - encoder->state.prev_position) / delta_time;
    encoder->state.prev_position = position;

    return err;
}

rotary_encoder_err_t rotary_encoder_get_acceleration(rotary_encoder_t* encoder,
                                                     float32_t* acceleration,
                                                     float32_t delta_time)
{
    assert(encoder && acceleration && delta_time > 0.0F);

    float32_t speed;
    rotary_encoder_err_t err = rotary_encoder_get_speed(encoder, &speed, delta_time);

    *acceleration = (speed - encoder->state.prev_speed) / delta_time;
    encoder->state.prev_speed = speed;

    return err;
}