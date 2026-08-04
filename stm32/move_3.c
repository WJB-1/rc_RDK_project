#include "move.h"

#include "brushless_motor.h"
#include "car_sport.h"
#include "slave.h"
#include "speed_PID.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(__GNUC__)
#pragma GCC optimize("Os")
#endif

extern int start;
float delta_distance=0;
/*
 * ============================================================
 * move.c 阅读指南
 * ============================================================
 *
 * 这个文件现在按功能分成 7 个模块：
 *
 * 1. 协议和工具函数
 *    负责校验和、大小端读写、角度归一化、基础限幅
 *
 * 2. 状态与反馈模块
 *    负责内部状态保存，以及里程/状态回包
 *
 * 3. 直行与普通巡航模块
 *    负责：
 *    - 上位机给 vx / wz 后，换算成左右轮速度
 *    - 新增：直行时的陀螺仪航向纠偏
 *
 * 4. 原地转向模块
 *    负责：
 *    - 收到转角命令后，记录当前 yaw
 *    - 根据 yaw 误差让左右轮反向转动
 *
 * 6. 协议解析与命令分发模块
 *    负责：
 *    - 从串口字节流里拆出完整帧
 *    - 根据命令字分发到不同执行函数
 *
 * 7. 对外接口模块
 *    负责：
 *    - 给 main.c 调用的初始化、20ms 任务、回包、急停、里程清零
 *
 * 你以后读代码时，可以直接按这 7 块往下找。
 */

/* ============================================================
 * 1. 协议和工具函数
 * ============================================================ */

/*
 * 这些参数主要分成 4 类：
 * 1. 底盘通用速度限幅
 * 2. 直行航向纠偏参数
 * 3. 原地转向参数
 * 4. 路口边走边转参数
 */

/* 左右轮目标速度的总上限，单位 cm/s */
static const float kWheelSpeedLimitCmps = 80.0f;
/* -------------------- 原地转向参数 -------------------- */

/* 原地转向结束时，允许的角度误差窗口，单位度 */
static const float kTurnFinishWindowDeg = 1.0f;

/* 原地转向结束时，允许的最大角速度，单位 deg/s */
static const float kTurnFinishGyroDps = 10.0f;

/* 原地转向完成判据需要连续满足的 20ms 周期数 */
static const uint8_t kTurnFinishStableTicks = 4U;

/* 原地转向外环 P 参数 */
static const float kTurnOuterLoopKp = 0.20f;

/* 原地转向外环 D 参数 */
static const float kTurnOuterLoopKd = 0.01f;

/* 原地转向时左右轮允许的最小速度，单位 cm/s */
static const float kTurnMinWheelSpeedCmps = 5.0f;

/* 原地转向时左右轮允许的最大速度，单位 cm/s */
static const float kTurnMaxWheelSpeedCmps = 18.0f;

/* 舵机默认动作时间，单位 ms */
static const float kDefaultServoMoveTimeMs = 300.0f;

/* -------------------- 路口边走边转参数 -------------------- */

/* 路口触发距离阈值 */
static const int16_t kIntersectionTriggerDistanceMm = 80;

/* 路口默认转角，当前固定 90 度 */
static const float kCornerTurnAngleDeg = 90.0f;

/* 边走边转结束时的角度误差窗口，单位度 */
static const float kCornerFinishWindowDeg = 1.0f;

/* 边走边转结束时允许的最大角速度，单位 deg/s */
static const float kCornerFinishGyroDps = 20.0f;

/* 边走边转完成判据需要连续满足的 20ms 周期数 */
static const uint8_t kCornerFinishStableTicks = 5U;

/* 边走边转外环 P 参数 */
static const float kCornerOuterLoopKp = 1.0f;

/* 边走边转外环 D 参数 */
static const float kCornerOuterLoopKd = 0.015f;

/* 边走边转时允许叠加的最大左右轮差速，单位 cm/s */
static const float kCornerMaxDeltaCmps = 50.0f;

/* 边走边转时内侧轮允许的最小速度，单位 cm/s */
static const float kCornerMinInnerWheelSpeedCmps = 0.0f;

/* 边走边转时若上位机没给速度，则默认以前进 20cm/s 转弯 */
static const float kCornerDefaultForwardSpeedCmps = 20.0f;


float vision_error = 0.0f;
float vision_correction = 0.0f;

/* 把浮点数限制在指定区间内 */
static float Move_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/* 把角度归一化到 [-180, 180) 区间，避免跨 180 度时误差突变 */
static float Move_NormalizeAngleDeg(float angle)
{
    while (angle>= 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/* 从小端格式的 2 个字节里读出 uint16 */
static uint16_t Move_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/* 从小端格式的 2 个字节里读出 int16 */
static int16_t Move_ReadI16LE(const uint8_t *data)
{
    return (int16_t)Move_ReadU16LE(data);
}

/* 把 32 位无符号整数按小端格式写入 4 个字节 */
static void Move_WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

/*
 * 协议校验和：
 * 前面所有字节累加，然后只取低 8 位。
 */
static uint8_t Move_Checksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0U;
    uint16_t i = 0U;

    for (i = 0U; i < len; ++i)
    {
        sum = (uint16_t)(sum + data[i]);
    }

    return (uint8_t)(sum & 0xFFU);
}

/* ============================================================
 * 2. 状态与反馈模块
 * ============================================================ */

/*
 * 这是 move 模块的内部上下文。
 * 你可以把它理解成：
 * “当前底盘控制状态的总表”。
 */
typedef struct
{
    UART_HandleTypeDef *upper_uart; /* 正式上位机串口句柄 */

    uint8_t rx_cache[MOVE_RX_CACHE_LEN]; /* 接收缓存：存放尚未完全解析的字节流 */
    uint16_t rx_cache_len;               /* 当前缓存里有效字节数 */

    uint8_t tx_queue[MOVE_TX_QUEUE_LEN][MOVE_TX_FRAME_LEN]; /* 回包发送队列 */
    uint8_t tx_length[MOVE_TX_QUEUE_LEN];                   /* 每一帧的真实长度 */
    uint8_t tx_head;                                        /* 队列写指针 */
    uint8_t tx_tail;                                        /* 队列读指针 */
    uint8_t tx_count;                                       /* 当前待发送帧数 */

    /*
     * 这两个量直接对应上位机 MOVE_VECTOR 命令里的参数：
     * linear_mm_s_cmd  = vx
     */
    int16_t linear_mm_s_cmd;

    bool cruise_enabled;     /* 是否处于普通巡航状态 */
    bool turn_active;        /* 是否处于原地转向状态 */
    bool corner_turn_active; /* 是否处于边走边转状态 */

    /* 原地转向和边走边转都会用到这组 yaw 状态 */
    float turn_start_yaw_deg;
    float turn_target_yaw_deg;

    /* 边走边转时继续前进的基础速度，单位 cm/s */
    float corner_forward_speed_cm_s;

    /* 连续满足“已经转完”判据的周期计数 */
    uint8_t turn_settle_count;

    /* 最近一次收到运动命令的时间戳，目前主要保留做状态追踪 */
    uint32_t last_motion_cmd_tick;

    /* 当前状态，防止反复上报同一个状态 */
    uint8_t current_status;

    /* 等待路口转向 */
    bool wait_corner_turn;

    /* 记录检测到路口时的里程 */
    float corner_start_distance_cm;

    /* 路口方向 */
    uint8_t corner_direction;
} MoveContext;

static MoveContext g_move;

/*
 * 把左右轮目标速度写进 target。
 * 注意：这里只是改“目标速度”，并不会直接发 CAN。
 */
static void Move_SetWheelSpeed(float left_cm_s, float right_cm_s)
{
    target.speed_L = Move_ClampFloat(left_cm_s, -kWheelSpeedLimitCmps, kWheelSpeedLimitCmps);
    target.speed_R = Move_ClampFloat(right_cm_s, -kWheelSpeedLimitCmps, kWheelSpeedLimitCmps);
    target.speed = (target.speed_L + target.speed_R) * 0.5f;
}

/* 把一帧反馈包放进发送队列 */
static void Move_EnqueueFeedback(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t *frame = NULL;
    uint8_t frame_len = 0U;

    if (payload_len > MOVE_MAX_PAYLOAD_LEN)
    {
        return;
    }

    if (g_move.tx_count >= MOVE_TX_QUEUE_LEN)
    {
        g_move.tx_tail = (uint8_t)((g_move.tx_tail + 1U) % MOVE_TX_QUEUE_LEN);
        g_move.tx_count--;
    }

    frame = g_move.tx_queue[g_move.tx_head];
    frame[0] = MOVE_FRAME_HEAD_0;
    frame[1] = MOVE_FRAME_HEAD_1;
    frame[2] = type;
    frame[3] = payload_len;

    if ((payload != NULL) && (payload_len > 0U))
    {
        memcpy(&frame[4], payload, payload_len);
    }

    frame_len = (uint8_t)(payload_len + 5U);
    frame[frame_len - 1U] = Move_Checksum(frame, (uint16_t)(frame_len - 1U));

    g_move.tx_length[g_move.tx_head] = frame_len;
    g_move.tx_head = (uint8_t)((g_move.tx_head + 1U) % MOVE_TX_QUEUE_LEN);
    g_move.tx_count++;
}

/* 更新状态，并在状态变化时回传一帧状态包 */
static void Move_SetStatus(uint8_t status)
{
    uint8_t payload[1];

    if (g_move.current_status == status)
    {
        return;
    }

    g_move.current_status = status;
    payload[0] = status;
    //数据包回传
    Move_EnqueueFeedback(FB_STATUS, payload, sizeof(payload));
}

/* 把当前整车累计里程打包成回包，单位 mm */
static void Move_QueueOdometry(void)
{
    uint8_t payload[8];
    int32_t odom_mm = (int32_t)lroundf(car_distance * 10.0f);
    int16_t yaw_01deg = (int16_t)lroundf(imuData.yaw * 10.0f);
    int16_t speed_mms = (int16_t)lroundf(target.speed * 10.0f);
    Move_WriteU32LE(payload, (uint32_t)odom_mm);
    payload[4] = (uint8_t)(yaw_01deg & 0xFF);
    payload[5] = (uint8_t)((yaw_01deg >> 8) & 0xFF);
    payload[6] = (uint8_t)(speed_mms & 0xFF);
    payload[7] = (uint8_t)((speed_mms >> 8) & 0xFF);
    Move_EnqueueFeedback(FB_ODOMETRY, payload, sizeof(payload));
}

/* ============================================================
 * 3. 直行与普通巡航模块
 * ============================================================ */



static float Move_CalcVisionCorrection(void)
{
    static float kp = 5.0f;

    vision_correction = kp * vision_error;

    vision_correction =
        Move_ClampFloat(
            vision_correction,
            -300.0f,
             300.0f);

    return vision_correction;
}
/*
 * 普通巡航速度换算
 *
 * 上位机发：
 * vx = 线速度（mm/s）

 *
 * 下位机换算：
 * 左轮 = vx - 差速项
 * 右轮 = vx + 差速项
 */
static void Move_ApplyCruiseVector(void)
{
    float linear_cm_s = (float)g_move.linear_mm_s_cmd * 0.1f;
    float angular_rad_s = 0.0f;
    float wheel_delta = 0.0f;
    angular_rad_s =Move_CalcVisionCorrection() * 0.001f;

    /* 只有纯直行命令才叠加视觉自动纠偏 */
    wheel_delta = angular_rad_s * (width * 0.5f);

    Move_SetWheelSpeed(linear_cm_s - wheel_delta, linear_cm_s + wheel_delta);

    if ((fabsf(target.speed_L) > 0.01f) || (fabsf(target.speed_R) > 0.01f))
    {
        start = 1;
    }
    else
    {
        start = 0;
    }
}

/* ============================================================
 * 4. 原地转向模块
 * ============================================================ */

/* 启动一次原地 IMU 转向 */
static void Move_StartTurn(int16_t angle_deg)
{
    g_move.turn_active = true;
    g_move.corner_turn_active = false;

    g_move.turn_settle_count = 0U;
    g_move.turn_start_yaw_deg = imuData.yaw;
    g_move.turn_target_yaw_deg = imuData.yaw + (float)angle_deg;
    target.turn_yaw_angle = (float)angle_deg;
    g_move.last_motion_cmd_tick = HAL_GetTick();

    PID_out_clear();
    start = 1;
    Move_SetStatus(MOVE_STATUS_TURNING);
}

/* ============================================================
 * 5. 路口边走边转模块
 * ============================================================ */

/* 没有明确给边走边转速度时，从当前巡航速度里推一个前进速度 */
static float Move_GetCornerForwardSpeedCmps(void)
{
    float speed_cm_s = fabsf((float)g_move.linear_mm_s_cmd * 0.1f);

    if (speed_cm_s < 0.1f)
    {
        speed_cm_s = fabsf(target.speed);
    }

    if (speed_cm_s < 0.1f)
    {
        speed_cm_s = kCornerDefaultForwardSpeedCmps;
    }

    return Move_ClampFloat(speed_cm_s, kCornerMinInnerWheelSpeedCmps, kWheelSpeedLimitCmps);
}

/* 开始执行“边直行边转弯”的 90 度转向 */
static void Move_StartIntersectionTurn(uint8_t direction)
{
    float signed_angle = (direction == MOVE_TURN_DIR_LEFT) ? kCornerTurnAngleDeg : -kCornerTurnAngleDeg;

    g_move.turn_active = false;
    g_move.corner_turn_active = true;

    g_move.turn_settle_count = 0U;
    g_move.turn_start_yaw_deg = imuData.yaw;
    g_move.turn_target_yaw_deg = imuData.yaw + signed_angle;
    g_move.corner_forward_speed_cm_s = Move_GetCornerForwardSpeedCmps();
    g_move.cruise_enabled = true;
    g_move.last_motion_cmd_tick = HAL_GetTick();
    target.turn_yaw_angle = signed_angle;

    PID_out_clear();
    start = 1;
    Move_SetStatus(MOVE_STATUS_TURNING);
}

/*
 * 路口边走边转命令格式：

 */
static void Move_HandleIntersectionTurn(const uint8_t *payload, uint8_t len)
{
    uint8_t direction = 0U;

    if (len < 1U)
    {
        return;
    }

    direction = payload[0];

    if ((direction != MOVE_TURN_DIR_LEFT) && (direction != MOVE_TURN_DIR_RIGHT))
    {
        return;
    }
    /* 记录当前里程 */
    g_move.corner_start_distance_cm = car_distance;

    /* 记录方向 */
    g_move.corner_direction = direction;

    /* 进入等待转向状态 */
    g_move.wait_corner_turn = true;

    g_move.cruise_enabled = true;
    /* 固定10cm/s前进 */
    g_move.linear_mm_s_cmd = 200;
    /* 立即应用 */
    Move_ApplyCruiseVector();

    start = 1;

}

/* ============================================================
 * 6. 协议解析与命令分发模块
 * ============================================================ */

/* 处理离散动作命令，例如急停和里程清零 */
static void Move_HandleAction(uint8_t action)
{
    switch (action)
    {
    case MOVE_ACTION_STOP:
        Move_EmergencyStop();
        break;

    case MOVE_ACTION_RESET_ODOM:
        Move_ResetOdometry();
        break;

    case MOVE_ACTION_START_UID:
      //  g_move.uid_reader_enabled = true;
        break;

    default:
        break;
    }
}

/* 舵机控制命令解析 */
static void Move_HandleServo(const uint8_t *payload, uint8_t len)
{
    uint8_t servo_id = 0U;
    float angle = 0.0f;
    float use_time_ms = kDefaultServoMoveTimeMs;

    if (len == 0U)
    {
        return;
    }

    if (len == 1U)
    {
        angle = (float)payload[0];
    }
    else
    {
        servo_id = payload[0];
        angle = (float)payload[1];

        if (len >= 4U)
        {
            use_time_ms = (float)Move_ReadU16LE(&payload[2]);
        }
    }

    if (servo_id > 1U)
    {
        return;
    }

    if (servo_id == 0U)
    {
        angle = Move_ClampFloat(angle, 0.0f, 270.0f);
    }
    else
    {
        angle = Move_ClampFloat(angle, 0.0f, 180.0f);
    }

    cloud_control_one((int)servo_id, angle, use_time_ms);
}

/* 命令分发器：根据命令字调用不同执行模块 */
static void Move_HandleFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd)
    {
    case CMD_MOVE_VECTOR:
        if (len >= 2U)
        {

            g_move.linear_mm_s_cmd = Move_ReadI16LE(&payload[0]);
            g_move.cruise_enabled = true;
            g_move.last_motion_cmd_tick = HAL_GetTick();

            /*
             * 如果当前不在任何转向状态，就立刻应用新的巡航目标。
             * 如果正在转向，就先把新命令缓存下来，等转完后自动恢复。
             */
            if ((!g_move.turn_active) && (!g_move.corner_turn_active))
            {
                Move_ApplyCruiseVector();
                Move_SetStatus(MOVE_STATUS_IDLE);
            }
        }
        break;

    case CMD_ACTION:
        if (len >= 1U)
        {
            Move_HandleAction(payload[0]);
        }
        break;

    case CMD_TURN_IMU:
        if (len >= 2U)
        {
            Move_StartTurn(Move_ReadI16LE(payload));
        }
        break;

    case CMD_SERVO:
        Move_HandleServo(payload, len);
        break;

    case CMD_INTERSECTION_TURN:
        Move_HandleIntersectionTurn(payload, len);
        break;
            case CMD_VISION_ERROR:
        {
            if(len >= 2)
            {
                vision_error =
                    (float)Move_ReadI16LE(payload);
            }
                break;
        }
    default:
        break;
    }
}


/*
 * 串口拆包器
 * 它负责：
 * 1. 在缓存里找帧头 A5 5A
 * 2. 读取长度
 * 3. 判断一帧是否收完整
 * 4. 校验和
 * 5. 调 Move_HandleFrame()
 */
static void Move_ParseCache(void)
{
    uint16_t frame_len = 0U;
    while (g_move.rx_cache_len >= 5U)
    {
        uint16_t i = 0U;
        bool found_header = false;
        uint8_t cmd = 0U;
        uint8_t payload_len = 0U;
        for (i = 0U; i + 1U < g_move.rx_cache_len; ++i)
        {
            if ((g_move.rx_cache[i] == MOVE_FRAME_HEAD_0) &&
                (g_move.rx_cache[i + 1U] == MOVE_FRAME_HEAD_1))
            {
                found_header = true;
                break;
            }
        }

        if (!found_header)
        {
            if (g_move.rx_cache[g_move.rx_cache_len - 1U] == MOVE_FRAME_HEAD_0)
            {
                g_move.rx_cache[0] = MOVE_FRAME_HEAD_0;
                g_move.rx_cache_len = 1U;
            }
            else
            {
                g_move.rx_cache_len = 0U;
            }
            return;
        }

        if (i > 0U)
        {
            memmove(g_move.rx_cache, &g_move.rx_cache[i], g_move.rx_cache_len - i);
            g_move.rx_cache_len = (uint16_t)(g_move.rx_cache_len - i);
        }

        if (g_move.rx_cache_len < 5U)
        {
            return;
        }

        cmd = g_move.rx_cache[2];
        payload_len = g_move.rx_cache[3];

        if (payload_len > MOVE_MAX_PAYLOAD_LEN)
        {
            memmove(g_move.rx_cache, &g_move.rx_cache[1], g_move.rx_cache_len - 1U);
            g_move.rx_cache_len--;
            continue;
        }

        frame_len = (uint16_t)(payload_len + 5U);
        if (g_move.rx_cache_len < frame_len)
        {
            return;
        }

        if (Move_Checksum(g_move.rx_cache, frame_len - 1U) != g_move.rx_cache[frame_len - 1U])
        {
            memmove(g_move.rx_cache, &g_move.rx_cache[1], g_move.rx_cache_len - 1U);
            g_move.rx_cache_len--;
            continue;
        }

        Move_HandleFrame(cmd, &g_move.rx_cache[4], payload_len);

        if (g_move.rx_cache_len > frame_len)
        {
            memmove(g_move.rx_cache, &g_move.rx_cache[frame_len], g_move.rx_cache_len - frame_len);
        }
        g_move.rx_cache_len = (uint16_t)(g_move.rx_cache_len - frame_len);
    }
}

/* ============================================================
 * 7. 对外接口模块
 * ============================================================ */

void Move_Init(UART_HandleTypeDef *upper_uart)
{
    memset(&g_move, 0, sizeof(g_move));
    g_move.upper_uart = upper_uart;
    g_move.current_status = 0xFFU; /* 这样首次状态一定会上报 */
    //清除里程
    Move_ResetOdometry();
    //取消当前命令
    Move_EmergencyStop();
    //目前处于空闲状态
    Move_SetStatus(MOVE_STATUS_IDLE);
}

void Move_InputBytes(const uint8_t *data, uint16_t size)
{
    if ((data == NULL) || (size == 0U))
    {
        return;
    }

    if (size >= MOVE_RX_CACHE_LEN)
    {
        data += (size - MOVE_RX_CACHE_LEN);
        size = MOVE_RX_CACHE_LEN;
        g_move.rx_cache_len = 0U;
    }

    if ((uint16_t)(g_move.rx_cache_len + size) > MOVE_RX_CACHE_LEN)
    {
        g_move.rx_cache_len = 0U;
    }

    memcpy(&g_move.rx_cache[g_move.rx_cache_len], data, size);
    g_move.rx_cache_len = (uint16_t)(g_move.rx_cache_len + size);
    Move_ParseCache();
}


/*
 * 每 20ms 运行一次：
 * 1. 若在原地转向，就优先执行原地转向控制
 * 2. 否则若在边走边转，就优先执行边走边转控制
 * 3. 否则若在普通巡航，就执行普通巡航/直行纠偏
 * 4. 最后统一回传里程
 */
void Move_Process20ms(void)
{
    if (g_move.turn_active)
    {
        float yaw_error_deg = Move_NormalizeAngleDeg(g_move.turn_target_yaw_deg - imuData.yaw);
        float yaw_rate_dps = imuData.gyro[0];
        float command_speed = 0.0f;

        if ((fabsf(yaw_error_deg) <= kTurnFinishWindowDeg) &&
            (fabsf(yaw_rate_dps) <= kTurnFinishGyroDps))
        {
            g_move.turn_settle_count++;
        }
        else
        {
            g_move.turn_settle_count = 0U;
        }

        if (g_move.turn_settle_count >= kTurnFinishStableTicks)
        {
            g_move.turn_active = false;
            g_move.turn_settle_count = 0U;
            PID_out_clear();
            Move_SetWheelSpeed(0.0f, 0.0f);
            start = 0;
            Move_SetStatus(MOVE_STATUS_TURN_DONE);
        }
        else
        {
            command_speed = kTurnOuterLoopKp * fabsf(yaw_error_deg) - kTurnOuterLoopKd * fabsf(yaw_rate_dps);
            command_speed = Move_ClampFloat(command_speed, kTurnMinWheelSpeedCmps, kTurnMaxWheelSpeedCmps);

            if (yaw_error_deg >= 0.0f)
            {
                /* 左转：左轮后退，右轮前进 */
                Move_SetWheelSpeed(-command_speed, command_speed);
            }
            else
            {
                /* 右转：左轮前进，右轮后退 */
                Move_SetWheelSpeed(command_speed, -command_speed);
            }

            start = 1;
        }
    }
    else if (g_move.corner_turn_active)
    {
        float yaw_error_deg = Move_NormalizeAngleDeg(g_move.turn_target_yaw_deg - imuData.yaw);
        float yaw_rate_dps = imuData.gyro[0];
        float turn_delta_cm_s = 0.0f;
        float left_speed = 0.0f;
        float right_speed = 0.0f;

        if ((fabsf(yaw_error_deg) <= kCornerFinishWindowDeg) &&
            (fabsf(yaw_rate_dps) <= kCornerFinishGyroDps))
        {
            g_move.turn_settle_count++;
        }
        else
        {
            g_move.turn_settle_count = 0U;
        }

        if (g_move.turn_settle_count >= kCornerFinishStableTicks)
        {
            /*
             * 边走边转结束后：
             * 1. 退出 corner_turn_active
             * 2. 把 wz 清零，恢复成普通直行
             * 3. 重新锁定当前 yaw，继续直行航向保持
             */
            g_move.corner_turn_active = false;
            g_move.turn_settle_count = 0U;


            Move_ApplyCruiseVector();
            Move_SetStatus(MOVE_STATUS_IDLE);
        }
        else
        {
            turn_delta_cm_s = kCornerOuterLoopKp * yaw_error_deg - kCornerOuterLoopKd * yaw_rate_dps;
            turn_delta_cm_s = Move_ClampFloat(turn_delta_cm_s, -kCornerMaxDeltaCmps, kCornerMaxDeltaCmps);

            left_speed = g_move.corner_forward_speed_cm_s - turn_delta_cm_s;
            right_speed = g_move.corner_forward_speed_cm_s + turn_delta_cm_s;

            /*
             * 保证这里真的是“边走边转”，而不是某一侧轮子倒车变成原地转。
             */
            left_speed = Move_ClampFloat(left_speed, kCornerMinInnerWheelSpeedCmps, kWheelSpeedLimitCmps);
            right_speed = Move_ClampFloat(right_speed, kCornerMinInnerWheelSpeedCmps, kWheelSpeedLimitCmps);

            Move_SetWheelSpeed(left_speed, right_speed);
            start = 1;
        }
    }
    else if (g_move.wait_corner_turn)
    {


        delta_distance =
            car_distance -
            g_move.corner_start_distance_cm;

        /* 继续保持当前巡航 */
        Move_ApplyCruiseVector();

        if (delta_distance >= kIntersectionTriggerDistanceMm)
        {
            g_move.wait_corner_turn = false;

            Move_StartIntersectionTurn(
                g_move.corner_direction);
        }
    }
    else if (g_move.cruise_enabled)
    {
        /*
         * 现在巡航已经改成“上位机发一次就保持当前状态”。
         * 所以这里不会再因为没持续收到命令而停车。
         */
        Move_ApplyCruiseVector();
    }


    Move_QueueOdometry();
}

void Move_UartTask(void)
{
    HAL_StatusTypeDef status;

    if ((g_move.upper_uart == NULL) || (g_move.tx_count == 0U))
    {
        return;
    }

    status = HAL_UART_Transmit(g_move.upper_uart,
                               g_move.tx_queue[g_move.tx_tail],
                               g_move.tx_length[g_move.tx_tail],
                               5U);

    if (status == HAL_OK)
    {
        g_move.tx_tail = (uint8_t)((g_move.tx_tail + 1U) % MOVE_TX_QUEUE_LEN);
        g_move.tx_count--;
    }
}

void Move_EmergencyStop(void)
{
    g_move.cruise_enabled = false;
    g_move.turn_active = false;
    g_move.corner_turn_active = false;

    g_move.linear_mm_s_cmd = 0;
    g_move.corner_forward_speed_cm_s = 0.0f;
    g_move.turn_settle_count = 0U;

    PID_out_clear();
    Move_SetWheelSpeed(0.0f, 0.0f);
    start = 0;
    Move_SetStatus(MOVE_STATUS_IDLE);
}

void Move_ResetOdometry(void)
{
    leftWheel.distance = 0.0f;
    rightWheel.distance = 0.0f;
    car_distance = 0.0f;
    target.position = 0.0f;
}

void Move_ReportSensorUid(const char *uid_str)
{
    uint8_t payload[17];  /* 最大 16 字符 + 1 长度字节 */
    uint8_t len = (uint8_t)strlen(uid_str);
    if (len > 16) len = 16;

    payload[0] = len;
    memcpy(&payload[1], uid_str, len);
    Move_EnqueueFeedback(FB_SENSOR, payload, 1 + len);
}

uint8_t Move_GetStatus(void)
{
    return g_move.current_status;
}


