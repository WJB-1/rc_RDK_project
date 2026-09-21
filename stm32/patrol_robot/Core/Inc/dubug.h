#ifndef PATROL_ROBOT_DUBUG_H
#define PATROL_ROBOT_DUBUG_H

#include "main.h"
#include "move.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Temporary USART1 commands for bench testing:
 *   '1' - enter cruise mode and drive forward at 30 cm/s
 *   '2' - stop
 *   '3' - reset odometry
 *   '4' - approach the intersection, then turn left while moving
 *   '5' - approach the intersection, then turn right while moving
 * CR/LF and spaces around a single command are accepted.
 */
static inline bool Dubug_HandleCommand(const uint8_t *data, uint16_t size)
{
    static const uint8_t cruise_30_cm_s_frame[] = {
        MOVE_FRAME_HEAD_0,
        MOVE_FRAME_HEAD_1,
        CMD_MOVE_VECTOR,
        2U,
        0x2CU, 0x01U, /* int16 little-endian: 300 mm/s */
        0x2FU         /* checksum */
    };
    static const uint8_t intersection_left_frame[] = {
        MOVE_FRAME_HEAD_0,
        MOVE_FRAME_HEAD_1,
        CMD_INTERSECTION_TURN,
        1U,
        MOVE_TURN_DIR_LEFT,
        0x06U /* checksum */
    };
    static const uint8_t intersection_right_frame[] = {
        MOVE_FRAME_HEAD_0,
        MOVE_FRAME_HEAD_1,
        CMD_INTERSECTION_TURN,
        1U,
        MOVE_TURN_DIR_RIGHT,
        0x07U /* checksum */
    };
    uint8_t command = 0U;
    bool command_found = false;
    uint16_t i;

    if ((data == NULL) || (size == 0U))
    {
        return false;
    }

    for (i = 0U; i < size; ++i)
    {
        if ((data[i] == ' ') || (data[i] == '\r') ||
            (data[i] == '\n') || (data[i] == '\t'))
        {
            continue;
        }

        if (command_found || (data[i] < '1') || (data[i] > '5'))
        {
            return false;
        }

        command = data[i];
        command_found = true;
    }

    if (!command_found)
    {
        return false;
    }

    switch (command)
    {
    case '1':
        Move_EmergencyStop();
        straight_imu_enabled = 1U;
        Move_InputBytes(cruise_30_cm_s_frame,
                        (uint16_t)sizeof(cruise_30_cm_s_frame));
        break;

    case '2':
        Move_EmergencyStop();
        break;

    case '3':
        Move_ResetOdometry();
        break;

    case '4':
        Move_EmergencyStop();
        straight_imu_enabled = 1U;
        Move_InputBytes(intersection_left_frame,
                        (uint16_t)sizeof(intersection_left_frame));
        break;

    case '5':
        Move_EmergencyStop();
        straight_imu_enabled = 1U;
        Move_InputBytes(intersection_right_frame,
                        (uint16_t)sizeof(intersection_right_frame));
        break;

    default:
        return false;
    }

    return true;
}

#endif /* PATROL_ROBOT_DUBUG_H */
