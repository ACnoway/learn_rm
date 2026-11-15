#ifndef CHASSIS_EXAMPLE_H
#define CHASSIS_EXAMPLE_H

#include <stdint.h>
#include <stdbool.h>

/* 四个数组位置始终按照左前、右前、左后、右后排列。 */
enum { FL = 0, FR, RL, RR, WHEEL_COUNT };

typedef struct {
    uint16_t angle;       /* 转子机械角度：0~8191 */
    int16_t rpm;          /* 电机转子转速，不是减速箱输出轴转速 */
    int16_t current;      /* 电调反馈的转矩电流原始值 */
    uint8_t temperature;  /* 摄氏度 */
    uint32_t last_rx_ms;  /* 最近一次有效反馈的时间 */
    bool received;
} MotorFeedback;

typedef struct {
    float kp, ki;
    float integral;       /* 积分项本身，单位与输出相同 */
    float integral_limit;
    float output_limit;   /* C620 电流指令的原始值限制 */
} SpeedPI;

typedef struct {
    float vx, vy, wz;     /* m/s、m/s、rad/s；车体坐标系 */
    uint32_t last_rx_ms;
    bool valid;           /* 协议解析、校验、范围检查均通过 */
    bool enable;          /* 用户明确使能 */
} ChassisCommand;

typedef struct {
    float wheel_radius;   /* 轮子有效半径，m */
    float half_length;    /* 车中心到前/后轮轴线的距离，m */
    float half_width;     /* 车中心到左/右轮中心线的距离，m */
    float gear_ratio;     /* 电机转子转速 / 轮子转速，含外部传动 */
    float max_motor_rpm;  /* 转子目标转速的绝对值上限 */
    int8_t motor_dir[WHEEL_COUNT]; /* 安装方向，仅允许 +1 或 -1 */
    uint32_t command_timeout_ms;
    uint32_t feedback_timeout_ms;
} ChassisConfig;

typedef struct {
    ChassisConfig config;
    MotorFeedback motor[WHEEL_COUNT];
    SpeedPI pi[WHEEL_COUNT];
    ChassisCommand command;
} Chassis;

/* 初始化为保守的教学占位值。必须按实车修改并整定后使用。 */
void chassis_init(Chassis *c);

/* 仅对 0x201~0x204 的标准数据帧、DLC=8 调用；外层检查帧类型。 */
bool chassis_on_can_feedback(Chassis *c, uint32_t std_id,
                             const uint8_t data[8], uint32_t now_ms);

/* 输入必须是经过协议校验、范围检查的完整快照。 */
void chassis_set_command(Chassis *c, const ChassisCommand *command);

/* 周期调用。始终生成 8 字节数据：有效运行时为控制电流，否则全零。
 * 返回 true 表示当前允许闭环；false 表示失能或故障。
 * 外层负责使用 CAN 标准数据帧 ID 0x200 发送，并检查发送结果。
 */
bool chassis_step(Chassis *c, uint32_t now_ms, float dt, uint8_t data[8]);

#endif
