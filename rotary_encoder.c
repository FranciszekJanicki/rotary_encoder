#include "rotary_encoder.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static inline rotary_encoder_err_t rotary_encoder_device_initialize(
    rotary_encoder_t const* encoder)
{
    return (encoder->interface.device_initialize != NULL)
               ? encoder->interface.device_initialize(
                     encoder->interface.device_user)
               : ROTARY_ENCODER_ERR_NULL;
}

static inline rotary_encoder_err_t rotary_encoder_device_deinitialize(
    rotary_encoder_t const* encoder)
{
    return (encoder->interface.device_deinitialize != NULL)
               ? encoder->interface.device_deinitialize(
                     encoder->interface.device_user)
               : ROTARY_ENCODER_ERR_NULL;
}

static inline rotary_encoder_err_t rotary_encoder_device_get_step_count(
    rotary_encoder_t const* encoder,
    int64_t* step_count)
{
    return (encoder->interface.device_get_step_count != NULL)
               ? encoder->interface.device_get_step_count(
                     encoder->interface.device_user,
                     step_count)
               : ROTARY_ENCODER_ERR_NULL;
}

static inline float32_t rotary_encoder_wrap_position(
    rotary_encoder_t const* encoder,
    float32_t position)
{
    float32_t position_range =
        fabsf(encoder->config.max_position - encoder->config.min_position);

    position = fmodf(position, position_range);
    while (position < encoder->config.min_position) {
        position += position_range;
    }
    if (position >= encoder->config.max_position) {
        position -= position_range;
    }

    return position;
}

static inline int64_t rotary_encoder_position_to_step_count(
    rotary_encoder_t const* encoder,
    float32_t position)
{
    if (encoder->config.step_change <= 0.0F) {
        return 0L;
    }

    float32_t step_count =
        rotary_encoder_wrap_position(encoder,position) / encoder->config.step_change;

    return (int64_t)roundf(step_count);
}

static inline float32_t rotary_encoder_step_count_to_position(
    rotary_encoder_t const* encoder,
    int64_t step_count)
{
    if (encoder->config.step_change <= 0.0F) {
        return 0.0F;
    }

    float32_t position = (float32_t)step_count * encoder->config.step_change;

    return rotary_encoder_wrap_position(encoder,position);
}

rotary_encoder_err_t rotary_encoder_initialize(
    rotary_encoder_t* encoder,
    rotary_encoder_config_t const* config,
    rotary_encoder_interface_t const* interface)
{
    if (encoder == NULL || config == NULL || interface == NULL) {
        return ROTARY_ENCODER_ERR_NULL;
    }

    memset(encoder, 0, sizeof(*encoder));
    memcpy(&encoder->config, config, sizeof(*config));
    memcpy(&encoder->interface, interface, sizeof(*interface));

    return rotary_encoder_device_initialize(encoder);
}

rotary_encoder_err_t rotary_encoder_deinitialize(rotary_encoder_t* encoder)
{
    if (encoder == NULL) {
        return ROTARY_ENCODER_ERR_NULL;
    }

    rotary_encoder_err_t err = rotary_encoder_device_deinitialize(encoder);
    if (err != ROTARY_ENCODER_ERR_OK) {
        return err;
    }

    memset(encoder, 0, sizeof(*encoder));

    return ROTARY_ENCODER_ERR_OK;
}

rotary_encoder_err_t rotary_encoder_reset(rotary_encoder_t* encoder)
{
    if (encoder == NULL) {
        return ROTARY_ENCODER_ERR_NULL;
    }

    memset(&encoder->state, 0, sizeof(encoder->state));

    return ROTARY_ENCODER_ERR_OK;
}

rotary_encoder_err_t rotary_encoder_get_position(rotary_encoder_t* encoder,
                                                 float32_t* position)
{
    if (encoder == NULL || position == NULL) {
        return ROTARY_ENCODER_ERR_NULL;
    }

    int64_t step_count;
    rotary_encoder_err_t err =
        rotary_encoder_device_get_step_count(encoder, &step_count);
    if (err != ROTARY_ENCODER_ERR_OK) {
        return err;
    }

    *position = rotary_encoder_step_count_to_position(encoder, step_count);

    return ROTARY_ENCODER_ERR_OK;
}

rotary_encoder_err_t rotary_encoder_get_speed(rotary_encoder_t* encoder,
                                              float32_t* speed,
                                              float32_t delta_time)
{
    if (encoder == NULL || speed == NULL) {
        return ROTARY_ENCODER_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return ROTARY_ENCODER_ERR_FAIL;
    }

    float32_t position;
    rotary_encoder_err_t err = rotary_encoder_get_position(encoder, &position);
    if (err != ROTARY_ENCODER_ERR_NULL) {
        return err;
    }

    *speed = (position - encoder->state.prev_position) / delta_time;
    encoder->state.prev_position = position;

    return ROTARY_ENCODER_ERR_OK;
}

rotary_encoder_err_t rotary_encoder_get_acceleration(rotary_encoder_t* encoder,
                                                     float32_t* acceleration,
                                                     float32_t delta_time)
{
    if (encoder == NULL || acceleration == NULL) {
        return ROTARY_ENCODER_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return ROTARY_ENCODER_ERR_FAIL;
    }

    float32_t speed;
    rotary_encoder_err_t err =
        rotary_encoder_get_speed(encoder, &speed, delta_time);
    if (err != ROTARY_ENCODER_ERR_NULL) {
        return err;
    }

    *acceleration = (speed - encoder->state.prev_speed) / delta_time;
    encoder->state.prev_speed = speed;

    return ROTARY_ENCODER_ERR_OK;
}