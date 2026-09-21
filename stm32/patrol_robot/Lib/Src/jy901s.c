/**
 * @file       JY901S.c
 * @brief      JY901S陀螺仪驱动实现（串口通信、数据解析、参数配置）
 * @author     ottohesl----zhujijun
 * @date       25-12-6
 * @version    V1.4
 * @note       适配STM32F1/F4/H7系列，基于HAL库开发，支持DMA接收/解析、参数校准/配置
 * |************************** 版本更新说明 ******************************|
 * @note   v1.1     较1.0新增局部陀螺仪结构体私有变量，使得外部可以声明jy901s的结构变量，多元化数据获取。
 *                  且初始化就可以直接囊括串口句柄和结构体句柄，更加清晰调用
 *         v1.2     将其结构体变量变为全局变量，无需在外部再添加结构变量，能直接使用头文件声明的结构体变量访问数据
 *         v1.3     添加预处理命令，使用文件前在头文件处将JY901S_H_Vision的版本改到你芯片所处版本即可
 *                  添加预处理命令，使用文件前在头文件将OTTOHELS文件进行包含管理
 *         v1.4     添加预处理命令，可以进行串口debug寻找具体错误原因
 */
#include "JY901S.h"

/************************ 宏定义 ************************/
#define G 9.80665f  // 重力加速度常量，单位m/s²

/********************** 调试模式开关 *********************/
#define DEBUG_MODE 0       //1开启，0关闭

/********************** 调试最大限制 *********************/
#define DEBUG_WIDTH 20

/************************ 缓冲区 ************************/
//stm32h7使用前必看--->>>>>>非h7等高端芯片可直接更改版本号
//开始之前将以下代码粘贴到本文件目录下的_FLASH.ld文件末尾处，并确认RAM区域是0x24000000开始的
/*
.sram_msg :
{
    . = ALIGN(4);
    *(.ram) // 匹配代码中的section名
    } > RAM //映射到0x24000000开始的SRAM
*/
#if JY901S_H_Vision==7
uint8_t RX[RX_SIZE] __attribute__((section(".ram"))); // DMA接收缓冲区（迁址到DMA可访问区域）
#else
uint8_t RX[RX_SIZE];
#endif
/************************ 全局变量 ************************/
UART_HandleTypeDef *huart_sensor;
UART_HandleTypeDef *huart_debugs;
jy901 gyro_data;       // 陀螺仪解析后的数据存储
/************************ 私有函数声明 ************************/
static void Gyroscope_Data(const uint8_t *data) ;        // 解析单帧陀螺仪原始数据
static int16_t Gyroscope_HL_Combine(uint8_t h,uint8_t l); // 高低字节合成16位有符号数
/**
 * @brief      修改JY901S串口波特率
 * @param      huart  串口句柄（对应JY901S连接的串口）
 * @retval     无
 * @note       1. 支持9600/115200/230400波特率，默认配置为115200
 *             2. 修改后需在CubeMX同步修改串口波特率，重新运行函数完成保存
 *             3. 指令流程：解锁→修改波特率→保存配置
 */
void Gyroscope_Alter_Bit(UART_HandleTypeDef *huart) {
    uint8_t unlock[5]={0xFF,0xAA,0x69,0x88,0xB5};          // 解锁配置指令
    uint8_t save[5]={0xFF,0xAA,0x00,0x00,0x00};             // 保存配置指令
    uint8_t change_bit_9600[5]={0xFF,0xAA,0x04,0x02,0x00};  // 配置波特率9600
    uint8_t change_bit_115200[5]={0xFF,0xAA,0x04,0x06,0x00};// 配置波特率115200
    uint8_t change_bit_230400[5]={0xFF,0xAA,0x04,0x07,0x00};// 配置波特率230400

    // 解锁配置权限
    HAL_UART_Transmit(huart,unlock,5,100);
    HAL_Delay(200);
    // 发送波特率修改指令（默认115200）
    HAL_UART_Transmit(huart,change_bit_115200,5,100);
    HAL_Delay(50);
    // 保存配置（修改生效）
    HAL_UART_Transmit(huart,save,5,100);
}

/**
 * @brief      配置JY901S数据输出速率
 * @param      huart  串口句柄（对应JY901S连接的串口）
 * @retval     无
 * @note       1. 支持2/5/10/50/100/200Hz，默认配置为100Hz
 *             2. 200Hz速率过高可能导致DMA接收不及时，推荐最大100Hz
 *             3. 指令流程：解锁→修改速率→保存配置
 */
void Gyroscope_Rrate(UART_HandleTypeDef *huart) {
    uint8_t unlock[5]={0xFF,0xAA,0x69,0x88,0xB5};              // 解锁配置指令
    uint8_t save[5]={0xFF,0xAA,0x00,0x00,0x00};                 // 保存配置指令
    uint8_t change_rate_2HZ[5]={0xFF,0xAA,0x03,0x03,0x00};      // 配置输出速率2Hz
    uint8_t change_rate_5HZ[5]={0xFF,0xAA,0x03,0x05,0x00};      // 配置输出速率5Hz
    uint8_t change_rate_10HZ[5]={0xFF,0xAA,0x03,0x06,0x00};     // 配置输出速率10Hz
    uint8_t change_rate_50HZ[5]={0xFF,0xAA,0x03,0x08,0x00};     // 配置输出速率50Hz
    uint8_t change_rate_100HZ[5]={0xFF,0xAA,0x03,0x09,0x00};    // 配置输出速率100Hz
    uint8_t change_rate_200HZ[5]={0xFF,0xAA,0x03,0x0B,0x00};    // 配置输出速率200Hz

    // 解锁配置权限
    HAL_UART_Transmit(huart,unlock,5,100);
    HAL_Delay(200);
    // 发送速率修改指令（默认100Hz）
    HAL_UART_Transmit(huart,change_rate_100HZ,5,100);
    HAL_Delay(100);
    // 保存配置（修改生效）
    HAL_UART_Transmit(huart,save,5,100);
}

/**
 * @brief      执行JY901S加速度计校准
 * @param      huart  串口句柄（对应JY901S连接的串口）
 * @retval     无
 * @note       1. 校准过程需保持陀螺仪静止，水平放置
 *             2. 指令流程：解锁→启动校准→延时4s（校准过程）→退出校准→保存配置
 *             3. 4秒延时为校准预留时间，不可缩短
 */
void Gyroscope_Accele_Calibra(UART_HandleTypeDef *huart) {
    uint8_t unlock[5]={0xFF,0xAA,0x69,0x88,0xB5};              // 解锁配置指令
    uint8_t save[5]={0xFF,0xAA,0x00,0x00,0x00};                 // 保存配置指令
    uint8_t change_data[5]={0xFF,0xAA,0x01,0x01,0x00};          // 启动加速度校准指令
    uint8_t exit_change_data[5]={0xFF,0xAA,0x01,0x00,0x00};     // 退出加速度校准指令

    // 解锁配置权限
    HAL_UART_Transmit(huart,unlock,5,100);
    HAL_Delay(200);
    // 启动加速度计校准
    HAL_UART_Transmit(huart,change_data,5,100);
    HAL_Delay(4000); // 校准耗时4秒，保持设备静止
    // 退出校准模式
    HAL_UART_Transmit(huart,exit_change_data,5,100);
    HAL_Delay(100);
    // 保存校准结果
    HAL_UART_Transmit(huart,save,5,100);
}

/**
 * @brief      执行JY901S陀螺仪校准
 * @param      huart  串口句柄（对应JY901S连接的串口）
 * @retval     无
 * @note       1. 校准过程需保持陀螺仪静止，水平放置
 *             2. 指令流程：解锁→启动校准→延时3s（校准过程）→退出校准→保存配置
 *             3. 3秒延时为校准预留时间，不可缩短
 */
void Gyroscope_Gyro_Calibra(UART_HandleTypeDef *huart) {
    uint8_t unlock[5]={0xFF,0xAA,0x69,0x88,0xB5};              // 解锁配置指令
    uint8_t save[5]={0xFF,0xAA,0x00,0x00,0x00};                 // 保存配置指令
    uint8_t change_data[5]={0xFF,0xAA,0x61,0x00,0x00};          // 启动陀螺仪校准指令
    uint8_t exit_change_data[5]={0xFF,0xAA,0x61,0x01,0x00};     // 退出陀螺仪校准指令

    // 解锁配置权限
    HAL_UART_Transmit(huart,unlock,5,100);
    HAL_Delay(200);
    // 启动陀螺仪校准
    HAL_UART_Transmit(huart,change_data,5,100);
    HAL_Delay(3000); // 校准耗时3秒，保持设备静止
    // 退出校准模式
    HAL_UART_Transmit(huart,exit_change_data,5,100);
    HAL_Delay(100);
    // 保存校准结果
    HAL_UART_Transmit(huart,save,5,100);
}

/**
 * @brief      启动JY901S DMA接收
 * @param      h_senor  串口句柄（对应JY901S连接的串口）
 * @param      h_debug  调试句柄（调试所需发送到的串口）
 * @retval     无
 * @note       1. 基于HAL库DMA接收接口，缓冲区为全局数组RX[RX_SIZE]
 *             2. 需确保RX缓冲区迁址到STM32H7 DMA可访问区域（0x24000000后）
 *             3. 调用一次即可持续DMA接收，无需重复调用
 */
void Gyroscope_Init(UART_HandleTypeDef *h_senor,UART_HandleTypeDef *h_debug) {
    huart_sensor = h_senor;
    huart_debugs = h_debug;
    HAL_StatusTypeDef Check_Error=HAL_UART_Receive_DMA(huart_sensor,RX,RX_SIZE);//一定要开启dma循环模式
#if DEBUG_MODE
    if (Check_Error!=HAL_OK) {
        ottohesl_uart(huart_debugs,"串口接受初始化错误");
    }
    if (h_senor->hdmarx->State!=HAL_DMA_STATE_READY) {
        ottohesl_uart(huart_debugs,"串口接受DMA未就绪，稍等");
        if(h_senor->hdmatx->State==HAL_DMA_STATE_RESET) {
            ottohesl_uart(huart_debugs,"DMA没有初始化");
        }else if (h_senor->hdmatx->State==HAL_DMA_STATE_BUSY) {
            ottohesl_uart(huart_debugs,"DMA忙碌中");
        }else if (h_senor->hdmatx->State==HAL_DMA_STATE_ERROR) {
            ottohesl_uart(huart_debugs,"DMA出错");
        }else if (h_senor->hdmatx->State==HAL_DMA_STATE_ABORT) {
            ottohesl_uart(huart_debugs,"DMA传输错误");
        }
    }
    #endif
}

/**
 * @brief      解析JY901S DMA接收的原始数据
 * @retval     bool  - true：解析到有效数据；false：无有效数据
 * @note        预处理命令调试模式说明:在连续n次循环中，
 * @note       1. 采用状态机解析帧头→帧类型→帧数据，带超时/校验保护
 *             2. 解析前关闭全局中断，防止DMA缓冲区数据污染
 *             3. 支持多帧连续解析，校验和错误帧自动丢弃
 */
bool Gyroscope_Process() {
    bool Available_Data = false;               // 有效数据标志
    uint32_t DMA_Received_Index = 0;           // DMA当前接收位置
    uint32_t DMA_Received_Length = 0;          // 本次待解析数据长度
    static uint8_t byte_pos = 0;               // 帧数据字节偏移
    static uint8_t checksum = 0;               // 帧校验和
    static uint16_t frame_timeout = 0;         // 帧解析超时计数器
    static const uint8_t TIMEOUT = 100;        // 帧解析超时阈值
    static uint8_t RX_Process[Frame_Length];   // 单帧数据临时缓冲区
    static uint32_t DMA_Received_Index_Last = 0;// 上一次解析位置
    static Frame_State frame_state = SEEK_FRAME_HEAD; // 帧解析状态机

    // 关闭全局中断，防止DMA缓冲区数据被篡改
    __disable_irq();
    // 获取DMA当前接收位置（剩余字节数反算已接收位置）
    DMA_Received_Index= RX_SIZE - __HAL_DMA_GET_COUNTER(huart_sensor->hdmarx);
    // 计算本次待解析的数据长度（处理缓冲区环形溢出）
    if (DMA_Received_Index>=DMA_Received_Index_Last) {
        DMA_Received_Length=DMA_Received_Index-DMA_Received_Index_Last;
    } else {
        DMA_Received_Length=RX_SIZE+DMA_Received_Index-DMA_Received_Index_Last;
    }
    // 恢复全局中断
    __enable_irq();
#if DEBUG_MODE
    static int error_count = 0;
    static uint32_t DMA_Received_Length_Debug = 0;
    //前者条件判断当前是否读取到新数据；后者条件判断是否下一次产生新数据，没有新数据则相减>=0，有新数据为负数，表明为有效数据
    if ((DMA_Received_Length==0)&&((DMA_Received_Length_Debug-DMA_Received_Length) >= 0)) error_count++;
    else error_count=0;
    if (error_count>DEBUG_WIDTH) {
        error_count=0;
        ottohesl_uart(huart_debugs,"原始数据错误：未收集到任何数据！");
    }
    DMA_Received_Length_Debug = DMA_Received_Length;
#endif
    // 有新数据时执行解析
    if (DMA_Received_Length>0) {
        for (uint32_t i=0;i<DMA_Received_Length;i++) {
            // 环形缓冲区取数，避免越界
            uint32_t byte=RX[(DMA_Received_Index_Last+i)%RX_SIZE];
            frame_timeout++; // 超时计数器递增

            // 帧解析超时保护：重置状态机
            if (frame_timeout>TIMEOUT) {
                checksum=0;
                byte_pos=0;
                frame_timeout=0;
                frame_state=SEEK_FRAME_HEAD;
                continue;
            }

            // 帧解析状态机
            switch (frame_state) {
                case SEEK_FRAME_HEAD: // 寻找帧头0x55
                    if (byte==Frame_Head) {
                        RX_Process[byte_pos++]=byte;
                        checksum=byte;
                        frame_timeout=0;
                        frame_state=SEEK_FRAME_TYPE;
                    }
                    break;

                case SEEK_FRAME_TYPE: // 寻找有效帧类型
                    if ((byte>=Frame_Accele&&byte<=Frame_Magnet) || byte==Frame_Quater) {
                        RX_Process[byte_pos++]=byte;
                        checksum+=byte;
                        frame_state=SEEK_FRAME_DATA;
                    } else {
                        // 无效帧类型，重置状态机
                        byte_pos=0;
                        checksum = 0;
                        frame_state=SEEK_FRAME_HEAD;
                    }
                    break;

                case SEEK_FRAME_DATA: // 接收帧数据并校验
                    RX_Process[byte_pos++]=byte;
                    if (byte_pos < Frame_Length) {
                        checksum += byte; // 累加校验和
                    }
                    // 单帧数据接收完成
                    if (byte_pos>=Frame_Length) {
                        // 校验和匹配：解析数据
                        if ((checksum&0xFF) == RX_Process[10]) {
                            Gyroscope_Data(RX_Process);
                            Available_Data= true;
                        }
                        // 重置状态机，准备解析下一帧
                        byte_pos=0;
                        checksum=0;
                        frame_timeout=0;
                        frame_state=SEEK_FRAME_HEAD;
                    }
                    break;
            }
        }
    }
    // 更新上一次解析位置
    DMA_Received_Index_Last=DMA_Received_Index;
    return Available_Data;
}

/**
 * @brief      解析单帧JY901S原始数据，转换为物理量
 * @param      data  单帧原始数据（长度=Frame_Length=11字节）
 * @retval     无
 * @note       1. 根据帧类型分别解析加速度/角速度/角度/磁场/四元数
 *             2. 原始数据为16位有符号数，需转换为物理量（带单位）
 *             3. 温度数据随加速度帧一并解析
 */
static void Gyroscope_Data(const uint8_t *data) {
    switch (data[1]) {
        case Frame_Accele: // 加速度+温度帧
            gyro_data.gyroscope.accele[0] = (float)Gyroscope_HL_Combine(data[3],data[2])/32768.0f * 16.0f * G;
            gyro_data.gyroscope.accele[1] = (float)Gyroscope_HL_Combine(data[5],data[4])/32768.0f * 16.0f * G;
            gyro_data.gyroscope.accele[2] = (float)Gyroscope_HL_Combine(data[7],data[6])/32768.0f * 16.0f * G;
            gyro_data.temp = (float)Gyroscope_HL_Combine(data[9],data[8])/100.0f;
            break;

        case Frame_Gyro: // 角速度帧
            gyro_data.gyroscope.gyro[0] = (float)Gyroscope_HL_Combine(data[3],data[2])/32768.0f * 2000;
            gyro_data.gyroscope.gyro[1] = (float)Gyroscope_HL_Combine(data[5],data[4])/32768.0f * 2000;
            gyro_data.gyroscope.gyro[2] = (float)Gyroscope_HL_Combine(data[7],data[6])/32768.0f * 2000;
            break;

        case Frame_Angle: // 角度帧
            gyro_data.gyroscope.angle[0] = (float)Gyroscope_HL_Combine(data[3],data[2])/32768.0f * 180.0f;
            gyro_data.gyroscope.angle[1] = (float)Gyroscope_HL_Combine(data[5],data[4])/32768.0f * 180.0f;
            gyro_data.gyroscope.angle[2] = (float)Gyroscope_HL_Combine(data[7],data[6])/32768.0f * 180.0f;
            break;

        case Frame_Magnet: // 磁场帧
            gyro_data.gyroscope.magnet[0] = (float)Gyroscope_HL_Combine(data[3],data[2])/150.0f;
            gyro_data.gyroscope.magnet[1] = (float)Gyroscope_HL_Combine(data[5],data[4])/150.0f;
            gyro_data.gyroscope.magnet[2] = (float)Gyroscope_HL_Combine(data[7],data[6])/150.0f;
            break;

        case Frame_Quater: // 四元数帧
            gyro_data.gyroscope.quaternion[0] = (float)Gyroscope_HL_Combine(data[3],data[2])/32768.0f;
            gyro_data.gyroscope.quaternion[1] = (float)Gyroscope_HL_Combine(data[5],data[4])/32768.0f;
            gyro_data.gyroscope.quaternion[2] = (float)Gyroscope_HL_Combine(data[7],data[6])/32768.0f;
            gyro_data.gyroscope.quaternion[3] = (float)Gyroscope_HL_Combine(data[9],data[8])/32768.0f;
            break;

        default:
            break;
    }
}

/**
 * @brief      高低字节合成16位有符号整数
 * @param      h  高8位字节
 * @param      l  低8位字节
 * @retval     int16_t  合成后的16位有符号数
 * @note       JY901S所有数据均为低字节在前、高字节在后，需按此规则合成
 */
static int16_t Gyroscope_HL_Combine(uint8_t h,uint8_t l) {
    return (int16_t)((uint16_t)h << 8 | l);
}

/**
 * @brief      发送解析后的陀螺仪数据（格式化输出）-----参考函数
 * @param      huart  串口句柄（用于数据发送的串口）
 * @retval     无
 * @note       1. 数据格式化为易读字符串，包含单位说明
 *             2. 当前为注释状态，可根据需求启用HAL_UART_Transmit发送
 *             3. 支持加速度/角速度/角度/温度/磁场/四元数全量发送
 */
void Gyroscope_Data_Send(UART_HandleTypeDef *huart) {
    const static int size=100; // 字符串缓冲区大小
    char Send_Date_accle[size];  // 加速度数据字符串
    char Send_Date_gyro[size];   // 角速度数据字符串
    char Send_Date_angle[size];  // 角度数据字符串
    char Send_Date_temp[size];   // 温度数据字符串
    char Send_Date_Magnet[size]; // 磁场数据字符串
    char Send_Date_Quater[size]; // 四元数数据字符串
    //ottohesl_uart(huart,"%.2f,%.2f,%.2f",gyro_data.gyroscope.angle[0],gyro_data.gyroscope.angle[1],gyro_data.gyroscope.angle[2]);
    // 格式化加速度数据
    int len_accle=sprintf(Send_Date_accle,"x加速度: %.2f，y加速度: %.2f，z加速度: %.2f\n",
        gyro_data.gyroscope.accele[0],gyro_data.gyroscope.accele[1],gyro_data.gyroscope.accele[2]);

    // 格式化角速度数据）
    int len_gyro=sprintf(Send_Date_gyro,"x角速度: %.2f，y角速度: %.2f，z角速度: %.2f\n",
        gyro_data.gyroscope.gyro[0],gyro_data.gyroscope.gyro[1],gyro_data.gyroscope.gyro[2]);

    // 格式化角度数据
    int len_angle=sprintf(Send_Date_angle,"翻滚角: %.2f，俯仰角: %.2f，航偏角: %.2f\n",
        gyro_data.gyroscope.angle[0],gyro_data.gyroscope.angle[1],gyro_data.gyroscope.angle[2]);

    // 格式化温度数据
    int len_temp=sprintf(Send_Date_temp,"温度: %.2f\n",gyro_data.temp);

    // 格式化磁场数据
    int len_magent=sprintf(Send_Date_Magnet,"x磁: %.2f，y磁: %.2f，z磁: %.2f\n",
        gyro_data.gyroscope.magnet[0],gyro_data.gyroscope.magnet[1],gyro_data.gyroscope.magnet[2]);

    // 格式化四元数数据
    int len_Quater=sprintf(Send_Date_Quater,"w: %.2f，x: %.2f，y: %.2f，z: %.2f\n",
        gyro_data.gyroscope.quaternion[0],gyro_data.gyroscope.quaternion[1],gyro_data.gyroscope.quaternion[2],gyro_data.gyroscope.quaternion[3]);

    // 启用以下代码可发送对应数据
    // HAL_UART_Transmit(huart,(uint8_t *)Send_Date_accle,len_accle,100);
    // HAL_UART_Transmit(huart,(uint8_t *)Send_Date_gyro,len_gyro,100);
    //HAL_UART_Transmit(huart,(uint8_t *)Send_Date_angle,len_angle,100);
    // HAL_UART_Transmit(huart,(uint8_t *)Send_Date_temp,len_temp,100);
    // HAL_UART_Transmit(huart,(uint8_t *)Send_Date_Magnet,len_magent,100);
    // HAL_UART_Transmit(huart,(uint8_t *)Send_Date_Quater,len_Quater,100);
}

