
#ifndef AFX_DEF_0898
#define AFX_DEF_0898


#define MAX_DATA_LEN 2048
#define SENSE_MAX_DATA_LEN 2048
#define ORDER_BUF_LEN 512
#define SENSE_BUF_LEN 512

#define ORDER_BUF_LEN2 2048
#define SENSE_BUF_LEN2 2048

#define FIRST_REV_BYTES 300


//Server IP

//#define PREDICTIVE_IP "172.16.13.50" 
#define PREDICTIVE_IP "127.0.0.1"

//#define DATAMAG_PORT 9001

#define PREDICTIVE_PORT 8001

//UDP

//#define ROB_CONTRL_IP "172.16.13.155" //中控IP
#define ROB_CONTRL_IP "172.16.13.155" //中控IP
#define ROB_TELE_IP "172.16.13.65" //遥操作控制端IP
#define ROB_VISION_IP "172.16.13.30" //视觉IP

//#define ROB_TELE_IP "172.16.13.50" //遥操作控制端IPS

//#define ROB_CONTRL_PORT 7500  //中控端口

//#define ROB_TELE_PORT 8500 //遥操作控制端口

#define ROB_CONTRL_PORT 5500  //中控端口

#define ROB_TELE_PORT 6500 //遥操作控制端口

#define ROB_VISION_PORT 8891//全局视觉伺服端口
#define ROB_VISION_PORT2 8890//手眼视觉伺服端口

//视觉计算机插卡
#define PLATFORM_PORT2 9898
#define SERVER_PLATFORM_IP2 "172.16.13.179"

//卫星2平台手控制
#define HAND_SERVER_IP "172.16.13.179"
//#define HAND_SERVER_IP "127.0.0.1"

#define HAND_COMMAND_PORT 9001
#define HAND_SENSE_PORT 9002

#define PackageHead 0x9876b1
#define PackageEnd 0x9876b2

#define THUMB_BRAKE	0
#define THUMB_RELEASE 0x8972CC01

#endif
