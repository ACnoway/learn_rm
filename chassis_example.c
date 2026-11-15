#include "chassis_example.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define C620_MAX_COMMAND 16384.0f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 无符号减法支持毫秒计数器回绕；超时阈值须远小于计数器周期。 */
static bool fresh(uint32_t now, uint32_t then, uint32_t timeout)
{
    return (uint32_t)(now - then) <= timeout;
}

void chassis_init(Chassis *c)
{
    memset(c, 0, sizeof(*c));

    /* 以下尺寸仅作公式演示，必须量取自己的底盘！ */
    c->config.wheel_radius = 0.076f;
    c->config.half_length = 0.20f;
    c->config.half_width = 0.20f;
    c->config.gear_ratio = 3591.0f / 187.0f;
    c->config.max_motor_rpm = 1000.0f; /* 架空调试示例，并非电机极限 */
    c->config.command_timeout_ms = 100;
    c->config.feedback_timeout_ms = 50;

    /* 只是占位，必须逐轮确定。不要假定镜像安装一定对应此数组。 */
    c->config.motor_dir[FL] = +1;
    c->config.motor_dir[FR] = -1;
    c->config.motor_dir[RL] = +1;
    c->config.motor_dir[RR] = -1;

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        /* 零增益使示例上电不会因 PID 自动产生电流；需逐轮整定。 */
        c->pi[i].kp = 0.0f;
        c->pi[i].ki = 0.0f;
        c->pi[i].integral_limit = 500.0f;
        c->pi[i].output_limit = 1000.0f; /* 协议值，非安培 */
    }
}

bool chassis_on_can_feedback(Chassis *c, uint32_t std_id,
                             const uint8_t data[8], uint32_t now_ms)
{
    if (std_id < 0x201U || std_id > 0x204U) return false;

    MotorFeedback *m = &c->motor[std_id - 0x201U];
    /* C620 高字节在前；转速和电流要解释为有符号数。 */
    m->angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    m->rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    m->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    m->temperature = data[6];
    m->last_rx_ms = now_ms;
    m->received = true;
    return true;
}

void chassis_set_command(Chassis *c, const ChassisCommand *command)
{
    c->command = *command;
}

static float pi_update(SpeedPI *p, float target, float measured, float dt)
{
    float error = target - measured;
    float proportional = p->kp * error;
    float candidate_i = clampf(p->integral + p->ki * error * dt,
                               -p->integral_limit, p->integral_limit);
    float candidate_output = proportional + candidate_i;

    /* 发生饱和时停止继续向饱和方向积分，但允许积分使输出退饱和。 */
    if ((candidate_output <= p->output_limit &&
         candidate_output >= -p->output_limit) ||
        (candidate_output > p->output_limit && error < 0.0f) ||
        (candidate_output < -p->output_limit && error > 0.0f)) {
        p->integral = candidate_i;
    }
    return clampf(proportional + p->integral,
                  -p->output_limit, p->output_limit);
}

static void put_i16_be(uint8_t *out, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    out[0] = (uint8_t)(bits >> 8);
    out[1] = (uint8_t)bits;
}

bool chassis_step(Chassis *c, uint32_t now_ms, float dt, uint8_t data[8])
{
    /* 默认发零电流；只有所有条件满足后才计算非零输出。 */
    memset(data, 0, 8);
    const ChassisConfig *cfg = &c->config;
    const ChassisCommand *cmd = &c->command;
    bool ok = cmd->valid && cmd->enable &&
              fresh(now_ms, cmd->last_rx_ms, cfg->command_timeout_ms) &&
              isfinite(cmd->vx) && isfinite(cmd->vy) && isfinite(cmd->wz) &&
              isfinite(dt) && dt > 0.0f && dt <= 0.02f &&
              isfinite(cfg->wheel_radius) && cfg->wheel_radius > 0.0f &&
              isfinite(cfg->half_length) && cfg->half_length >= 0.0f &&
              isfinite(cfg->half_width) && cfg->half_width >= 0.0f &&
              isfinite(cfg->gear_ratio) && cfg->gear_ratio > 0.0f &&
              isfinite(cfg->max_motor_rpm) && cfg->max_motor_rpm > 0.0f;

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        if (!c->motor[i].received ||
            !fresh(now_ms, c->motor[i].last_rx_ms, cfg->feedback_timeout_ms) ||
            (cfg->motor_dir[i] != 1 && cfg->motor_dir[i] != -1) ||
            !isfinite(c->pi[i].kp) || c->pi[i].kp < 0.0f ||
            !isfinite(c->pi[i].ki) || c->pi[i].ki < 0.0f ||
            !isfinite(c->pi[i].integral) ||
            !isfinite(c->pi[i].integral_limit) || c->pi[i].integral_limit < 0.0f ||
            !isfinite(c->pi[i].output_limit) || c->pi[i].output_limit < 0.0f ||
            c->pi[i].output_limit > C620_MAX_COMMAND) {
            ok = false;
        }
    }

    if (!ok) {
        /* 失能后清积分，避免再次使能时沿用旧积分造成突跳。 */
        for (int i = 0; i < WHEEL_COUNT; ++i) c->pi[i].integral = 0.0f;
        return false;
    }

    /* X 型、45°滚子示例；四轮正值均定义为产生向前驱动趋势。 */
    float L = cfg->half_length + cfg->half_width;
    float wheel_mps[WHEEL_COUNT] = {
        cmd->vx - cmd->vy - L * cmd->wz, /* FL */
        cmd->vx + cmd->vy + L * cmd->wz, /* FR */
        cmd->vx + cmd->vy - L * cmd->wz, /* RL */
        cmd->vx - cmd->vy + L * cmd->wz  /* RR */
    };
    float factor = 60.0f * cfg->gear_ratio / (2.0f * PI_F * cfg->wheel_radius);
    float target[WHEEL_COUNT];
    float max_abs = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        target[i] = wheel_mps[i] * factor; /* 轮缘 m/s -> 电机转子 rpm */
        if (!isfinite(target[i])) {
            for (int j = 0; j < WHEEL_COUNT; ++j) c->pi[j].integral = 0.0f;
            return false;
        }
        if (fabsf(target[i]) > max_abs) max_abs = fabsf(target[i]);
    }
    if (max_abs > cfg->max_motor_rpm) {
        float scale = cfg->max_motor_rpm / max_abs;
        for (int i = 0; i < WHEEL_COUNT; ++i) target[i] *= scale;
    }

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        /* 在逻辑坐标下闭环，反馈和输出各转换一次方向。 */
        float logical_rpm = cfg->motor_dir[i] * (float)c->motor[i].rpm;
        float logical_current = pi_update(&c->pi[i], target[i], logical_rpm, dt);
        float raw_current = cfg->motor_dir[i] * logical_current;
        raw_current = clampf(raw_current, -C620_MAX_COMMAND, C620_MAX_COMMAND);
        put_i16_be(&data[2 * i], (int16_t)raw_current);
    }
    return true;
}
