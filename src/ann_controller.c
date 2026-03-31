#include "controller.h"
#include "log.h"
#include "usec_time.h"
#include "param.h"
#include "stabilizer_types.h"
#include "network.h"
#include "debug.h"
#include "motors.h"
#include "watchdog.h"
#include "math3d.h"
#include "crtp.h"
#include "commander.h"
#include "supervisor.h"
#include <math.h>
#include <string.h>
#include "controller_pid.h"
#include "ann_controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "crtp_commander_high_level.h"

#define NN_INTERVAL_TICKS 10 
#define NN_INTERVAL_US (NN_INTERVAL_TICKS * 1000)
#define CONTROL_PACKET_TIMEOUT_US (1000 * 500) // 500 ms 
#define BEHIND_SCHEDULE_MSG_US 1000000 // 1 s 
#define ANN_RAW_STATE_DIM 13 
#define ANN_OBS_DIM 18 
#define ANN_ACTION_DIM 4
#define ANN_ACTION_HISTORY_LEN  32
#define ANN_INPUT_DIM (ANN_OBS_DIM + ANN_ACTION_HISTORY_LEN * ANN_ACTION_DIM)  // 146
#define ANN_OUTPUT_DIM ANN_ACTION_DIM
#define WARMUP_TIME_US 500000
#define DEBUG_MODULE "ANN_CONTROLLER"

enum ControllerState {
    CTRL_STATE_IDLE = 0,
    CTRL_STATE_PID_TAKEOFF = 1,
    CTRL_STATE_ANN = 2,
    CTRL_STATE_PID_LANDING = 3
};

enum Mode {
    MODE_NORMAL = 0,
    MODE_HOVER = 1,
    MODE_FIGURE_EIGHT = 2,
    MODE_LANDING = 3,
    MODE_STOP = 4
};

typedef struct {
    uint8_t current;
    uint8_t mode;
    uint8_t prev_mode;
    uint64_t last_nn_start;
    uint64_t last_control_packet;
    uint64_t last_behind_msg;
    uint64_t controller_activation;
    uint64_t takeoff;
    uint64_t landing;
    uint64_t figureeight_last_invoke;
    uint64_t figureeight_start_time;
    uint64_t warmup_time;
    float figureeight_progress;
    bool control_link_active;
    bool prev_control_link_active;
    bool prev_set_motors;
    bool hlc_cmd_sent;
    bool run_ann_now;
    uint8_t log_motors_active;

} controller_context_t;

typedef struct {
    float target_pos[3];
    float target_vel[3];
    float pos_error[3];
    float relative_pos[3];
    float origin[3];
    float hover_pos[3];
    float fig8_center[3];
    float nn_time_us;
    float controller_time_us;
} controller_telemetry_t;

typedef struct {
    float target_height;
    float takeoff_height;
    float takeoff_speed;
    float landing_speed;
    float pos_limit;
    float vel_limit;
    float figureeight_period;
    float figureeight_warmup_time;
    float figureeight_scale;
    float velocity_cmd_multiplier;
    float velocity_cmd_gain;
} controller_params_t;

STAI_ALIGNED(STAI_NETWORK_CONTEXT_ALIGNMENT)
static stai_network network_context[STAI_NETWORK_CONTEXT_SIZE];

STAI_ALIGNED(STAI_NETWORK_ACTIVATION_1_ALIGNMENT)
static uint8_t activations[STAI_NETWORK_ACTIVATION_1_SIZE_BYTES];

static float state_raw[ANN_RAW_STATE_DIM]; 
static float ann_input[ANN_INPUT_DIM]; 
static float action_output[ANN_OUTPUT_DIM]; 
static float action_history[ANN_ACTION_HISTORY_LEN][ANN_ACTION_DIM];
static uint16_t motor_cmd[4];

static controller_context_t ctx;
static controller_telemetry_t telem;
static controller_params_t params;

/*========
Helpers
========*/

static inline float clip(float v, float low, float high) {
    if (v < low)  return low;
    if (v > high) return high;
    return v;
}

static inline void update_telemetry(const state_t* state) {
    telem.relative_pos[0] = state->position.x - telem.origin[0];
    telem.relative_pos[1] = state->position.y - telem.origin[1];
    telem.relative_pos[2] = state->position.z - telem.origin[2];

    telem.pos_error[0] = telem.target_pos[0] - state->position.x;
    telem.pos_error[1] = telem.target_pos[1] - state->position.y;
    telem.pos_error[2] = telem.target_pos[2] - state->position.z;
}

static inline void build_raw_state(const sensorData_t* sensors, const state_t* state) {
    float plim = params.pos_limit;
    state_raw[0] = clip(state->position.x - telem.target_pos[0], -plim, plim);
    state_raw[1] = clip(state->position.y - telem.target_pos[1] , -plim, plim);
    state_raw[2] = clip(state->position.z - telem.target_pos[2] , -plim, plim);

    state_raw[3] = state->attitudeQuaternion.w;
    state_raw[4] = state->attitudeQuaternion.x;
    state_raw[5] = state->attitudeQuaternion.y;
    state_raw[6] = state->attitudeQuaternion.z;

    float vlim = params.vel_limit;
    state_raw[7] = clip(state->velocity.x - telem.target_vel[0], -vlim, vlim);
    state_raw[8] = clip(state->velocity.y - telem.target_vel[1], -vlim, vlim);
    state_raw[9] = clip(state->velocity.z - telem.target_vel[2], -vlim, vlim);

    state_raw[10] = radians(sensors->gyro.x);
    state_raw[11] = radians(sensors->gyro.y);
    state_raw[12] = radians(sensors->gyro.z);
}

static void build_nn_input(void) {
    ann_input[0] = state_raw[0];
    ann_input[1] = state_raw[1];
    ann_input[2] = state_raw[2];

    float qw = state_raw[3], qx = state_raw[4], qy = state_raw[5], qz = state_raw[6];

    float r00 = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;
    float r01 = 2.0f*qx*qy - 2.0f*qw*qz;
    float r02 = 2.0f*qx*qz + 2.0f*qw*qy;
    float r10 = 2.0f*qx*qy + 2.0f*qw*qz;
    float r11 = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;
    float r12 = 2.0f*qy*qz - 2.0f*qw*qx;
    float r20 = 2.0f*qx*qz - 2.0f*qw*qy;
    float r21 = 2.0f*qy*qz + 2.0f*qw*qx;
    float r22 = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

    ann_input[3]  = r00; ann_input[4]  = r01; ann_input[5]  = r02;
    ann_input[6]  = r10; ann_input[7]  = r11; ann_input[8]  = r12;
    ann_input[9]  = r20; ann_input[10] = r21; ann_input[11] = r22;

    float vx_global = state_raw[7];
    float vy_global = state_raw[8];
    float vz_global = state_raw[9];

    ann_input[12] = r00 * vx_global + r10 * vy_global + r20 * vz_global;
    ann_input[13] = r01 * vx_global + r11 * vy_global + r21 * vz_global;
    ann_input[14] = r02 * vx_global + r12 * vy_global + r22 * vz_global;

    ann_input[15] = state_raw[10];
    ann_input[16] = state_raw[11];
    ann_input[17] = state_raw[12];

    int idx = ANN_OBS_DIM;
    for (int i = 0; i < ANN_ACTION_HISTORY_LEN; i++) {
        for (int a = 0; a < ANN_ACTION_DIM; a++) {
            ann_input[idx++] = action_history[i][a];
        }
    }
}
static void push_action_history(void) {
    memmove(&action_history[0][0], &action_history[1][0], (ANN_ACTION_HISTORY_LEN - 1) * ANN_ACTION_DIM * sizeof(float));
    memcpy(&action_history[ANN_ACTION_HISTORY_LEN - 1][0], action_output, ANN_ACTION_DIM * sizeof(float));
}

static void ann_heartbeat_callback(CRTPPacket* pk) {
    ctx.last_control_packet = usecTimestamp();
}

static bool process_state_machine(uint64_t now, const state_t* state) {

    if (ctx.mode == MODE_STOP) {
        if (ctx.current != CTRL_STATE_IDLE) {
            supervisorRequestArming(false);
            DEBUG_PRINT("emergency stop triggered\n");
        }
        ctx.current = CTRL_STATE_IDLE;
        ctx.hlc_cmd_sent = false;
        ctx.prev_mode = ctx.mode;
        return false; 
    }

    bool set_motors = false;

    if (ctx.mode == MODE_LANDING && ctx.prev_mode != MODE_LANDING
        && (ctx.current == CTRL_STATE_ANN || ctx.current == CTRL_STATE_PID_TAKEOFF)) {
        ctx.landing = now;
        ctx.current = CTRL_STATE_PID_LANDING;
        ctx.hlc_cmd_sent = false;
    }

    if (ctx.mode != ctx.prev_mode) {
        if (ctx.mode == MODE_HOVER) {
            telem.hover_pos[0] = telem.target_pos[0];
            telem.hover_pos[1] = telem.target_pos[1];
            telem.hover_pos[2] = telem.target_pos[2];
        } else if (ctx.mode == MODE_FIGURE_EIGHT) {
            telem.fig8_center[0] = telem.target_pos[0];
            telem.fig8_center[1] = telem.target_pos[1];
            telem.fig8_center[2] = telem.target_pos[2];
            ctx.figureeight_progress = 0.0f; 
            ctx.figureeight_last_invoke = now;
            ctx.figureeight_start_time = now;
        }
    }

    ctx.prev_mode = ctx.mode;

    switch (ctx.current) {
        case CTRL_STATE_IDLE:
            telem.nn_time_us = 0.0f;
            set_motors = false;
            break;

        case CTRL_STATE_PID_TAKEOFF:
            telem.nn_time_us = 0.0f;
            set_motors = true;
            
            if (((now - ctx.warmup_time) >= WARMUP_TIME_US) && !ctx.hlc_cmd_sent) {
                int rc = crtpCommanderHighLevelTakeoffWithVelocity(params.takeoff_height, params.takeoff_speed, true);
                if (rc == 0) {
                    ctx.takeoff = now;
                    ctx.hlc_cmd_sent = true;
                }
            }
            if (ctx.hlc_cmd_sent && crtpCommanderHighLevelIsTrajectoryFinished()) {

                ctx.controller_activation = now;
                ctx.figureeight_last_invoke = now;
                ctx.figureeight_progress = 0.0f;
                telem.hover_pos[0] = telem.origin[0];
                telem.hover_pos[1] = telem.origin[1];
                telem.hover_pos[2] = telem.origin[2] + params.takeoff_height;
                ctx.run_ann_now = true;
                ctx.last_nn_start = usecTimestamp();
                ctx.current = CTRL_STATE_ANN;
                DEBUG_PRINT("ANN active, mode=%d\n", ctx.mode);
            }
            break;

        case CTRL_STATE_ANN:
            set_motors = true;
            break;

        case CTRL_STATE_PID_LANDING:
            telem.nn_time_us = 0.0f;
            set_motors = true;
            if (!ctx.hlc_cmd_sent) {
                int rc = crtpCommanderHighLevelLandWithVelocity(0.0f, params.landing_speed, false);
                if (rc == 0) {
                    ctx.hlc_cmd_sent = true;
                    ctx.landing = now;
                    DEBUG_PRINT("landing...\n");
                }
            }
            if (ctx.hlc_cmd_sent && (crtpCommanderHighLevelIsTrajectoryFinished())) {
                supervisorRequestArming(false);
                ctx.current = CTRL_STATE_IDLE;
                ctx.hlc_cmd_sent = false;
                ctx.mode = MODE_HOVER;
                DEBUG_PRINT("Landing complete\n");
            }
            break;

        default:
            set_motors = false;
            ctx.current = CTRL_STATE_IDLE;
            break;
    }

    return set_motors;
}

static void update_targets(const setpoint_t* setpoint, const state_t* state, uint64_t now) {
    telem.target_vel[0] = 0.0f;
    telem.target_vel[1] = 0.0f;
    telem.target_vel[2] = 0.0f;

    switch (ctx.mode) {
        case MODE_NORMAL:
            if (setpoint->mode.x == modeAbs) {
                telem.target_pos[0] = setpoint->position.x;  
            } else if (setpoint->mode.x == modeVelocity) {
                telem.target_pos[0] = state->position.x - setpoint->velocity.x * params.velocity_cmd_gain;
                telem.target_vel[0] = setpoint->velocity.x * params.velocity_cmd_multiplier;
            } else {
                telem.target_pos[0] = telem.origin[0];  
            }
            if (setpoint->mode.y == modeAbs) {
                telem.target_pos[1] = setpoint->position.y;  
            } else if (setpoint->mode.y == modeVelocity) {
                telem.target_pos[1] = state->position.y - setpoint->velocity.y * params.velocity_cmd_gain;
                telem.target_vel[1] = setpoint->velocity.y * params.velocity_cmd_multiplier;
            } else {
                telem.target_pos[1] = telem.origin[1];  
            }
            if (setpoint->mode.z == modeAbs) {
                telem.target_pos[2] = setpoint->position.z;  
            } else if (setpoint->mode.z == modeVelocity) {
                telem.target_pos[2] = state->position.z - setpoint->velocity.z * params.velocity_cmd_gain;
                telem.target_vel[2] = setpoint->velocity.z * params.velocity_cmd_multiplier;
            } else {
                telem.target_pos[2] = telem.origin[2];  
            }
            break;

        case MODE_HOVER: 
            telem.target_pos[0] = telem.hover_pos[0];
            telem.target_pos[1] = telem.hover_pos[1];
            telem.target_pos[2] = telem.hover_pos[2];
            break;

        case MODE_FIGURE_EIGHT: {
            float t  = (now - ctx.figureeight_start_time) / 1000000.0f;
            float dt = (now - ctx.figureeight_last_invoke) / 1000000.0f;
            float tgt_spd = 1.0f / params.figureeight_period;
            float spd = (t < params.figureeight_warmup_time) ? tgt_spd * t / params.figureeight_warmup_time : tgt_spd;
            ctx.figureeight_progress += dt * spd;
            float p = ctx.figureeight_progress;
            
            telem.target_pos[0] = telem.fig8_center[0] + cosf(p*2*M_PI_F + M_PI_F/2) * params.figureeight_scale;
            telem.target_vel[0] = -sinf(p*2*M_PI_F + M_PI_F/2) * params.figureeight_scale * 2*M_PI_F * spd;

            telem.target_pos[1] = telem.fig8_center[1] + sinf(2*(p*2*M_PI_F + M_PI_F/2)) / 2.0f * params.figureeight_scale;
            telem.target_vel[1] = cosf(2*(p*2*M_PI_F + M_PI_F/2)) / 2.0f * params.figureeight_scale * 4*M_PI_F * spd;

            telem.target_pos[2] = telem.fig8_center[2]; // Mantiene l'altezza del centro registrato

            ctx.figureeight_last_invoke = now;
            break;
        }

        default:
            telem.target_pos[0] = telem.origin[0];
            telem.target_pos[1] = telem.origin[1];
            telem.target_pos[2] = telem.origin[2];
            break;
    }
    
    
}

/* ============
Controller API                                                              
==============*/

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    while(1) {
        vTaskDelay(M2T(2000));
    }
}

void controllerOutOfTreeInit(void) {
    stai_return_code ret_code;
    
    ret_code = stai_runtime_init();
    if (ret_code != STAI_SUCCESS) {
        DEBUG_PRINT("stai_runtime_init failed (%d)\n", ret_code);
        return;
    }
    
    ret_code = stai_network_init(network_context);
    if (ret_code != STAI_SUCCESS) {
        DEBUG_PRINT("stai_network_init failed (%d)\n", ret_code);
        return;
    }
    
    const stai_ptr acts[] = { activations };
    ret_code = stai_network_set_activations(network_context, acts, STAI_NETWORK_ACTIVATIONS_NUM);
    if (ret_code != STAI_SUCCESS) {
        DEBUG_PRINT("stai_network_set_activations failed (%d)\n", ret_code);
        return;
    }

    crtpRegisterPortCB(9, ann_heartbeat_callback);
    memset(motor_cmd, 0, sizeof(motor_cmd));
    memset(action_history, 0, sizeof(action_history));
    memset(&ctx, 0, sizeof(ctx));
    memset(&telem, 0, sizeof(telem));
    ctx.mode = MODE_HOVER;
    params.target_height = 0.3f;
    params.takeoff_height = 0.3f;
    params.takeoff_speed = 0.5f;
    params.landing_speed = 0.3f;
    params.pos_limit = 0.6f;
    params.vel_limit = 2.0f;
    params.figureeight_period = 5.5f;
    params.figureeight_scale = 1.0f;
    params.figureeight_warmup_time = 2.0f;
    params.velocity_cmd_multiplier = 1.0f;
    params.velocity_cmd_gain = 0.0f;
    ctx.current = CTRL_STATE_IDLE;
    crtpCommanderHighLevelInit();
    controllerPidInit();
    DEBUG_PRINT("Controller init\n");
}

bool controllerOutOfTreeTest(void) {
    memset(ann_input, 0, sizeof(ann_input));
    const stai_ptr in[]  = { (stai_ptr)ann_input };
    const stai_ptr out[] = { (stai_ptr)action_output };
    
    stai_network_set_inputs(network_context, in,  STAI_NETWORK_IN_NUM);
    stai_network_set_outputs(network_context, out, STAI_NETWORK_OUT_NUM);
    
    stai_return_code ret_code = stai_network_run(network_context, STAI_MODE_SYNC);
    if (ret_code != STAI_SUCCESS) {
        DEBUG_PRINT("Self-test FAILED (%d)\n", ret_code);
        return false;
    }
    
    DEBUG_PRINT("Self-test OK  [%.4f, %.4f, %.4f, %.4f]\n",
                (double)action_output[0], (double)action_output[1],
                (double)action_output[2], (double)action_output[3]);

    return true && controllerPidTest();
}

void controllerOutOfTree(control_t* control, const setpoint_t* setpoint, const sensorData_t* sensors, const state_t* state, const stabilizerStep_t stabilizerStep) {
    
    uint64_t start_ctrl_time = usecTimestamp();
    uint64_t now = usecTimestamp();

    watchdogReset();

    ctx.control_link_active = ((now - ctx.last_control_packet) < CONTROL_PACKET_TIMEOUT_US);

    if (!ctx.prev_control_link_active && ctx.control_link_active) {
      ctx.warmup_time = now;
      ctx.controller_activation = now;
      telem.origin[0] = state->position.x;
      telem.origin[1] = state->position.y;
      telem.origin[2] = state->position.z;
      ctx.figureeight_last_invoke = now;
      ctx.figureeight_progress = 0.0f;
      supervisorRequestArming(true);
      ctx.hlc_cmd_sent = false;   
      ctx.run_ann_now = false;
      ctx.current = CTRL_STATE_PID_TAKEOFF;
      DEBUG_PRINT("Takeoff\n");
    }

    if (ctx.prev_control_link_active && !ctx.control_link_active) {

      if (ctx.current != CTRL_STATE_IDLE) {
            ctx.current = CTRL_STATE_PID_LANDING;
            ctx.hlc_cmd_sent = false;
            ctx.run_ann_now = false;
        }
    }

    bool set_motors = process_state_machine(now, state);

    if (!set_motors) {
        if (ctx.prev_set_motors) {
            DEBUG_PRINT("DISARMED\n");
        }
        for (int i = 0; i < 4; i++) {
            control->normalizedForces[i] = 0.0f;
            motor_cmd[i] = 0;
        }
        control->controlMode = controlModeForce;
        ctx.prev_set_motors = false;
        ctx.prev_control_link_active = ctx.control_link_active;
        ctx.log_motors_active = 0;
        telem.controller_time_us = (float)(usecTimestamp() - start_ctrl_time);
        return; 
    }

    if (!ctx.prev_set_motors && set_motors) {
      ctx.controller_activation = now;
      ctx.figureeight_last_invoke = now;
      ctx.figureeight_progress = 0.0f;
      DEBUG_PRINT("ARMED  mode=%d\n", ctx.mode);
    }
    
    if (ctx.current == CTRL_STATE_PID_TAKEOFF || ctx.current == CTRL_STATE_PID_LANDING) {
        if (((now -ctx.warmup_time)< WARMUP_TIME_US) && (ctx.current == CTRL_STATE_PID_TAKEOFF) ) {
            for (int i = 0; i < 4; i++) {
                control->normalizedForces[i] = 0.35f;
            }
            control->controlMode = controlModeForce;
            ctx.hlc_cmd_sent = false; 
            
            ctx.prev_set_motors = set_motors;
            ctx.prev_control_link_active = ctx.control_link_active;
            telem.controller_time_us = (float)(usecTimestamp() - start_ctrl_time);
            return;
        }
        else{
            controllerPid(control, setpoint, sensors, state, stabilizerStep);
            ctx.prev_set_motors = set_motors;
            ctx.prev_control_link_active = ctx.control_link_active;
            ctx.log_motors_active = 1;
            telem.controller_time_us = (float)(usecTimestamp() - start_ctrl_time); 
            return;
        }
    }

    ctx.log_motors_active = set_motors ? 1 : 0; // should always be 1

    // Actual contrller that runs at 100Hz
    if (ctx.current == CTRL_STATE_ANN && ((stabilizerStep % NN_INTERVAL_TICKS == 0) || ctx.run_ann_now)) {
        
        update_telemetry(state);
        update_targets(setpoint, state, now);
        
        build_raw_state(sensors, state);
        build_nn_input();

        const stai_ptr in_ptr[]  = {(stai_ptr)ann_input};
        const stai_ptr out_ptr[] = {(stai_ptr)action_output};
        stai_network_set_inputs(network_context, in_ptr, STAI_NETWORK_IN_NUM);
        stai_network_set_outputs(network_context, out_ptr, STAI_NETWORK_OUT_NUM);

        uint64_t start_inference = usecTimestamp();

        stai_return_code rc = stai_network_run(network_context, STAI_MODE_SYNC);
        
        uint64_t end_inference = usecTimestamp();
        telem.nn_time_us = (float)(end_inference - start_inference);

        ctx.run_ann_now = false;

        if (rc != STAI_SUCCESS) {
            DEBUG_PRINT("inference error %d\n", rc);
        }

        push_action_history();

        int64_t elapsed = (int64_t)(now - ctx.last_nn_start);
        if (elapsed > NN_INTERVAL_US && (now - ctx.last_behind_msg > BEHIND_SCHEDULE_MSG_US)) {
            DEBUG_PRINT("Ann behind schedule: %lldus / %dus\n", (long long)elapsed, NN_INTERVAL_US);
            ctx.last_behind_msg = now;
        }

        ctx.last_nn_start = usecTimestamp();
    }

    if (set_motors) {
        for (int i = 0; i < ANN_ACTION_DIM; i++) {
            float action_01 = (action_output[i] + 1.0f) / 2.0f;
            motor_cmd[i] = (uint16_t)(action_01 * (float)UINT16_MAX);
            control->normalizedForces[i] = action_01;
        }
        control->controlMode = controlModeForce;
    } 

    ctx.prev_set_motors = set_motors;
    ctx.prev_control_link_active = ctx.control_link_active;
    telem.controller_time_us = (float)(usecTimestamp() - start_ctrl_time);
}


PARAM_GROUP_START(ann)
  PARAM_ADD(PARAM_UINT8, mode, &ctx.mode)
  PARAM_ADD(PARAM_FLOAT, takeoff_h, &params.takeoff_height)
  PARAM_ADD(PARAM_FLOAT, fig_eight_period, &params.figureeight_period)
  PARAM_ADD(PARAM_FLOAT, fig_eight_scale, &params.figureeight_scale)
  PARAM_ADD(PARAM_FLOAT, fig_eight_warmup_time, &params.figureeight_warmup_time)
  PARAM_ADD(PARAM_FLOAT, vcmdm, &params.velocity_cmd_multiplier)
  PARAM_ADD(PARAM_FLOAT, vcmdp, &params.velocity_cmd_gain)
PARAM_GROUP_STOP(ann)

LOG_GROUP_START(ann)
  LOG_ADD(LOG_FLOAT, rel_x,  &telem.relative_pos[0])
  LOG_ADD(LOG_FLOAT, rel_y,  &telem.relative_pos[1])
  LOG_ADD(LOG_FLOAT, rel_z,  &telem.relative_pos[2])
  LOG_ADD(LOG_FLOAT, tgt_x, &telem.target_pos[0])
  LOG_ADD(LOG_FLOAT, tgt_y, &telem.target_pos[1])
  LOG_ADD(LOG_FLOAT, tgt_z, &telem.target_pos[2])
  LOG_ADD(LOG_UINT8, motors_spinning, &ctx.log_motors_active)
  LOG_ADD(LOG_FLOAT, err_x, &telem.pos_error[0])
  LOG_ADD(LOG_FLOAT, err_y, &telem.pos_error[1])
  LOG_ADD(LOG_FLOAT, err_z, &telem.pos_error[2])
  LOG_ADD(LOG_FLOAT, nn_time, &telem.nn_time_us)
  LOG_ADD(LOG_FLOAT, ctrl_time, &telem.controller_time_us)
    LOG_ADD(LOG_UINT16, cmdpwm_m1, &motor_cmd[0])
    LOG_ADD(LOG_UINT16, cmdpwm_m2, &motor_cmd[1])
    LOG_ADD(LOG_UINT16, cmdpwm_m3, &motor_cmd[2])
    LOG_ADD(LOG_UINT16, cmdpwm_m4, &motor_cmd[3])
LOG_GROUP_STOP(ann)