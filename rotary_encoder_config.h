#ifndef ROTARY_ENCODER_ROTARY_ENCODER_CONFIG_H
#define ROTARY_ENCODER_ROTARY_ENCODER_CONFIG_H

#include <stdint.h>

#ifndef FLOAT32
#define FLOAT32
typedef float float32_t;
#endif // FLOAT32

typedef enum {
    ROTARY_ENCODER_ERR_OK = 0,
    ROTARY_ENCODER_ERR_FAIL = 1 << 0,
    ROTARY_ENCODER_ERR_NULL = 1 << 1,
} rotary_encoder_err_t;

typedef struct {
    int64_t step_count;
    float32_t prev_position;
    float32_t prev_speed;
} rotary_encoder_state_t;

typedef struct {
    float32_t step_change;
    float32_t min_position;
    float32_t max_position;
} rotary_encoder_config_t;

typedef struct {
    void* device_user;
    rotary_encoder_err_t (*device_init)(void*);
    rotary_encoder_err_t (*device_deinit)(void*);
    rotary_encoder_err_t (*device_get_step_count)(void*, int64_t*);
} rotary_encoder_interface_t;

#endif // ROTARY_ENCODER_ROTARY_ENCODER_CONFIG_H