#ifndef PATROL_ROBOT_COMMUNICATION_H
#define PATROL_ROBOT_COMMUNICATION_H

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#define COMM_VERSION        0x01U
#define COMM_SOF0           0xC3U
#define COMM_SOF1           0x3CU
#define COMM_MAX_PAYLOAD   16U
#define COMM_RX_CACHE_LEN  64U
#define COMM_TX_QUEUE_LEN   8U
#define COMM_TX_FRAME_LEN  (COMM_MAX_PAYLOAD + 7U)

typedef enum { COMM_TYPE_HELLO=0x01U, COMM_TYPE_ACTION=0x10U, COMM_TYPE_VISION_OFFSET=0x11U,
               COMM_TYPE_HELLO_ACK=0x81U, COMM_TYPE_ACTION_ACK=0x90U,
               COMM_TYPE_ACTION_DONE=0x91U, COMM_TYPE_REJECT=0x92U } CommunicationFrameType;
typedef enum { COMM_LINK_NEGOTIATING=0U, COMM_LINK_IDLE, COMM_LINK_RUNNING } CommunicationLinkState;
typedef enum { COMM_ACTION_CORRECT=0x01U, COMM_ACTION_TURN_LEFT=0x02U, COMM_ACTION_TURN_RIGHT=0x03U,
               COMM_ACTION_STRAIGHT=0x04U, COMM_ACTION_STOP=0x05U } CommunicationAction;
typedef enum { COMM_RESULT_SUCCESS=0x00U, COMM_RESULT_STOPPED=0x01U,
               COMM_RESULT_ACTUATOR_FAULT=0x02U, COMM_RESULT_VISION_TIMEOUT=0x03U } CommunicationActionResult;
typedef enum { COMM_REJECT_BUSY=0x01U, COMM_REJECT_INVALID_ACTION=0x02U,
               COMM_REJECT_NOT_READY=0x03U } CommunicationRejectReason;

void Communication_Init(UART_HandleTypeDef *uart);
void Communication_InputBytes(const uint8_t *data, uint16_t size);
void Communication_UartTask(void);
void Communication_Process20ms(void);
void Communication_ActionDone(uint8_t action, uint8_t result);
CommunicationLinkState Communication_GetLinkState(void);

#endif /* PATROL_ROBOT_COMMUNICATION_H */
