#include "communication.h"
#include "move.h"
#include <string.h>

extern bool Move_CommunicationStartAction(uint8_t action, const uint8_t *params, uint8_t params_len);
extern void Move_CommunicationControl(int16_t offset_mm);
extern void Move_CommunicationAbort(void);

typedef struct { UART_HandleTypeDef *uart; uint8_t rx[COMM_RX_CACHE_LEN]; uint16_t rx_len;
    uint8_t tx[COMM_TX_QUEUE_LEN][COMM_TX_FRAME_LEN]; uint8_t tx_len[COMM_TX_QUEUE_LEN];
    uint8_t tx_head, tx_tail, tx_count; CommunicationLinkState state; uint8_t action; uint32_t last_vision_ms; } CommunicationContext;
static CommunicationContext g_comm;

static uint16_t Comm_ReadU16LE(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint16_t Comm_Crc16(const uint8_t *data, uint16_t len)
{ uint16_t crc=0xFFFFU, i; uint8_t bit; for(i=0;i<len;++i){crc^=(uint16_t)data[i]<<8;for(bit=0;bit<8;++bit)crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);}return crc; }
static void Comm_QueueFrame(uint8_t type,const uint8_t *payload,uint8_t len)
{ uint8_t *f; uint16_t crc; if(len>COMM_MAX_PAYLOAD)return; if(g_comm.tx_count>=COMM_TX_QUEUE_LEN){g_comm.tx_tail=(uint8_t)((g_comm.tx_tail+1U)%COMM_TX_QUEUE_LEN);--g_comm.tx_count;} f=g_comm.tx[g_comm.tx_head]; f[0]=COMM_SOF0;f[1]=COMM_SOF1;f[2]=COMM_VERSION;f[3]=type;f[4]=len;if(payload&&len)memcpy(&f[5],payload,len);crc=Comm_Crc16(&f[2],(uint16_t)(len+3U));f[5U+len]=(uint8_t)crc;f[6U+len]=(uint8_t)(crc>>8);g_comm.tx_len[g_comm.tx_head]=(uint8_t)(len+7U);g_comm.tx_head=(uint8_t)((g_comm.tx_head+1U)%COMM_TX_QUEUE_LEN);++g_comm.tx_count; }
static void Comm_SendReply(uint8_t type,uint8_t action,uint8_t value)
{ uint8_t p[2]={action,value}; Comm_QueueFrame(type,p,(type==COMM_TYPE_ACTION_ACK)?1U:2U); }

static void Comm_HandleFrame(uint8_t type,const uint8_t *p,uint8_t len)
{ uint8_t action; switch(type){
case COMM_TYPE_HELLO: if(len!=0U)return; g_comm.state=COMM_LINK_IDLE; Comm_QueueFrame(COMM_TYPE_HELLO_ACK,NULL,0U); break;
case COMM_TYPE_ACTION:
    if (len == 0U)
        return;
    action = p[0];
    if(action==COMM_ACTION_STOP){if(len!=1U){Comm_SendReply(COMM_TYPE_REJECT,action,COMM_REJECT_INVALID_ACTION);return;} Move_CommunicationAbort();g_comm.state=COMM_LINK_IDLE;g_comm.action=action;Comm_SendReply(COMM_TYPE_ACTION_DONE,action,COMM_RESULT_STOPPED);return;}
    if(g_comm.state!=COMM_LINK_IDLE){Comm_SendReply(COMM_TYPE_REJECT,action,(g_comm.state==COMM_LINK_RUNNING)?COMM_REJECT_BUSY:COMM_REJECT_NOT_READY);return;}
    if(!Move_CommunicationStartAction(action,&p[1],(uint8_t)(len-1U))){Comm_SendReply(COMM_TYPE_REJECT,action,COMM_REJECT_INVALID_ACTION);return;}
    g_comm.action=action;g_comm.state=COMM_LINK_RUNNING;g_comm.last_vision_ms=HAL_GetTick();Comm_SendReply(COMM_TYPE_ACTION_ACK,action,0U); break;
case COMM_TYPE_VISION_OFFSET:
    if((g_comm.state!=COMM_LINK_RUNNING)||(g_comm.action!=COMM_ACTION_CORRECT)||(len!=2U))return;
    Move_CommunicationControl((int16_t)Comm_ReadU16LE(p));g_comm.last_vision_ms=HAL_GetTick(); break;
default: break; } }

static void Comm_Parse(void)
{ uint16_t frame_len,got,expected,i; uint8_t len; while(g_comm.rx_len>=2U){for(i=0U;i+1U<g_comm.rx_len;++i)if(g_comm.rx[i]==COMM_SOF0&&g_comm.rx[i+1U]==COMM_SOF1)break;if(i+1U>=g_comm.rx_len){if(g_comm.rx[g_comm.rx_len-1U]==COMM_SOF0){g_comm.rx[0]=COMM_SOF0;g_comm.rx_len=1U;}else g_comm.rx_len=0U;return;}if(i){memmove(g_comm.rx,&g_comm.rx[i],g_comm.rx_len-i);g_comm.rx_len=(uint16_t)(g_comm.rx_len-i);}if(g_comm.rx_len<5U)return;len=g_comm.rx[4];if(g_comm.rx[2]!=COMM_VERSION||len>COMM_MAX_PAYLOAD){memmove(g_comm.rx,&g_comm.rx[1],--g_comm.rx_len);continue;}frame_len=(uint16_t)(len+7U);if(g_comm.rx_len<frame_len)return;got=(uint16_t)g_comm.rx[5U+len]|((uint16_t)g_comm.rx[6U+len]<<8);expected=Comm_Crc16(&g_comm.rx[2],(uint16_t)(len+3U));if(got==expected)Comm_HandleFrame(g_comm.rx[3],&g_comm.rx[5],len);memmove(g_comm.rx,&g_comm.rx[frame_len],g_comm.rx_len-frame_len);g_comm.rx_len=(uint16_t)(g_comm.rx_len-frame_len);} }
void Communication_Init(UART_HandleTypeDef *uart){memset(&g_comm,0,sizeof(g_comm));g_comm.uart=uart;g_comm.state=COMM_LINK_NEGOTIATING;}
void Communication_InputBytes(const uint8_t *data,uint16_t size){if(!data||!size)return;if(size>COMM_RX_CACHE_LEN){data+=size-COMM_RX_CACHE_LEN;size=COMM_RX_CACHE_LEN;g_comm.rx_len=0U;}if((uint16_t)(g_comm.rx_len+size)>COMM_RX_CACHE_LEN)g_comm.rx_len=0U;memcpy(&g_comm.rx[g_comm.rx_len],data,size);g_comm.rx_len=(uint16_t)(g_comm.rx_len+size);Comm_Parse();}
void Communication_UartTask(void){if(!g_comm.uart||!g_comm.tx_count)return;if(HAL_UART_Transmit(g_comm.uart,g_comm.tx[g_comm.tx_tail],g_comm.tx_len[g_comm.tx_tail],5U)==HAL_OK){g_comm.tx_tail=(uint8_t)((g_comm.tx_tail+1U)%COMM_TX_QUEUE_LEN);--g_comm.tx_count;}}
void Communication_Process20ms(void){if(g_comm.state==COMM_LINK_RUNNING&&g_comm.action==COMM_ACTION_CORRECT&&(uint32_t)(HAL_GetTick()-g_comm.last_vision_ms)>200U){Move_CommunicationAbort();g_comm.state=COMM_LINK_IDLE;Comm_SendReply(COMM_TYPE_ACTION_DONE,COMM_ACTION_CORRECT,COMM_RESULT_VISION_TIMEOUT);}}
void Communication_ActionDone(uint8_t action,uint8_t result){if(g_comm.state!=COMM_LINK_RUNNING||action!=g_comm.action)return;g_comm.state=COMM_LINK_IDLE;Comm_SendReply(COMM_TYPE_ACTION_DONE,action,result);}
CommunicationLinkState Communication_GetLinkState(void){return g_comm.state;}
