#ifndef _SOCKETDEFINE_H 
#define _SOCKETDEFINE_H
 
#define MAX_DATA_LEN 2048
#define SENSE_MAX_DATA_LEN 2048
#define ORDER_BUF_LEN 512
#define SENSE_BUF_LEN 512

#define ORDER_BUF_LEN2 2048
#define SENSE_BUF_LEN2 2048

#define FIRST_REV_BYTES 300

#define VISION_DATA_LEN 56


//UDP

#define ROB_VISION_IP "172.16.13.30" //视觉IP
#define ROB_VISION_PORT 8891//全局视觉伺服端口
#define ROB_VISION_PORT2 8890//手眼视觉伺服端口

//视觉计算机插卡
#define PLATFORM_PORT2 9898
#define SERVER_PLATFORM_IP2 "172.16.13.179"

//卫星2平台手控制
//#define HAND_SERVER_IP "127.0.0.1"






//*********************** In Use ***********************//
// 连接宇航员
#define ROB_CONTRL_IP "172.16.13.155" // 中控IP
#define ROB_CONTRL_PORT 6500 // 中控端口

#define ROB_TELE_IP "172.16.13.65" // 遥操作控制端IP
#define ROB_TELE_PORT 5500  // 遥操作控制端口

// 连接预测仿真
#define PREDICTIVE_IP "127.0.0.1"		// 预测仿真IP
#define PREDICTIVE_PORT 8001			// 预测仿真端口

// 连接灵巧手
#define HAND_SERVER_IP "172.16.13.179"		// 灵巧手IP
#define HAND_COMMAND_PORT 9001		// 灵巧手接收控制命令端口
#define HAND_SENSE_PORT 9002		// 灵巧手发送传感器数据端口

#define PackageHead 0x9876b1		// 灵巧手数据包头
#define PackageEnd 0x9876b2			// 灵巧手数据包尾
#define THUMB_BRAKE	0				// 
#define THUMB_RELEASE 0x8972CC01

// 连接相机
#define VISION_SENSE_IP "127.0.0.1"
#define VISION_SENSE_PORT 6000



#endif
