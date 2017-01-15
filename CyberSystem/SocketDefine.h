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

#define ROB_VISION_IP "172.16.13.30" //�Ӿ�IP
#define ROB_VISION_PORT 8891//ȫ���Ӿ��ŷ��˿�
#define ROB_VISION_PORT2 8890//�����Ӿ��ŷ��˿�

//�Ӿ�������忨
#define PLATFORM_PORT2 9898
#define SERVER_PLATFORM_IP2 "172.16.13.179"

//����2ƽ̨�ֿ���
//#define HAND_SERVER_IP "127.0.0.1"






//*********************** In Use ***********************//
// �����Ա
#define ROB_CONTRL_IP "172.16.13.155" // �п�IP
#define ROB_CONTRL_PORT 6500 // �пض˿�

#define ROB_TELE_IP "172.16.13.65" // ң�������ƶ�IP
#define ROB_TELE_PORT 5500  // ң�������ƶ˿�

// ����Ԥ�����
#define PREDICTIVE_IP "127.0.0.1"		// Ԥ�����IP
#define PREDICTIVE_PORT 8001			// Ԥ�����˿�

// ����������
#define HAND_SERVER_IP "172.16.13.179"		// ������IP
#define HAND_COMMAND_PORT 9001		// �����ֽ��տ�������˿�
#define HAND_SENSE_PORT 9002		// �����ַ��ʹ��������ݶ˿�

#define PackageHead 0x9876b1		// ���������ݰ�ͷ
#define PackageEnd 0x9876b2			// ���������ݰ�β
#define THUMB_BRAKE	0				// 
#define THUMB_RELEASE 0x8972CC01

// �������
#define VISION_SENSE_IP "127.0.0.1"
#define VISION_SENSE_PORT 6000



#endif
