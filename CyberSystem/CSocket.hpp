#ifndef _CSOCKET_H
#define _CSOCKET_H



#include <WinSock2.h>
//#include <iostream>
#include <QMessageBox>

#pragma comment(lib, "ws2_32.lib")

namespace sockconn{
	// UDP Client
	class CUdpClient{
	private:
		SOCKET m_socket;
		SOCKADDR_IN m_SendAddr;
		WSABUF wsaSendBuf;

		bool m_bConStus;
		bool m_bInitStus;
	public:
		CUdpClient()
		{
			// Status initialize
			m_bConStus = false;	
			m_bInitStus = false;

			WSADATA wsaData;

			int err = WSAStartup(0x202, &wsaData);

			if (err != 0)
			{
				//std::cout << "WSAStartup failed with error: " << err;
				QMessageBox::critical(NULL, "Error", "WSAStartup failed with error");
				m_bConStus = false;
			}
			else{
				m_bConStus = true;
			}
		}
		~CUdpClient()
		{
			closesocket(m_socket);
		}
	public:
		bool Init(const char* IP, const unsigned short port)
		{
			if (m_bConStus == true)
			{
				// initialize socket
				m_socket = WSASocket(AF_INET,SOCK_DGRAM,IPPROTO_UDP,NULL,0,WSA_FLAG_OVERLAPPED);
				if (m_socket == INVALID_SOCKET)
				{
					//std::cout << "Socket create error" << std::endl;
					QMessageBox::critical(NULL, "Error", "Socket Create Error");
					return false;
				}

				// initialize address	
				m_SendAddr.sin_addr.S_un.S_addr = inet_addr(IP);
				m_SendAddr.sin_family = AF_INET;
				m_SendAddr.sin_port = htons(port);

				m_bInitStus = true;

				return true;
			}
			else{
				QMessageBox::critical(NULL, "Error", "WSAStartup Initialize Failed, ReInitialize!");
			}
		}

		bool CloseSock()
		{
			int ret = closesocket(m_socket);
			if (ret == 0)
			{
				return true;
			}
			// TODO(CJH): Add error lists here, to judge which error occurs
			else{
				return false;
			}
		}

		bool Send(char CmdBuf[], int CmdBuf_Len)
		{
			if (m_bInitStus == true)
			{
				DWORD dFlag = 0;
				DWORD BytesofSend;
				int iLen = sizeof(SOCKADDR_IN);  
				wsaSendBuf.buf = CmdBuf;
				wsaSendBuf.len = CmdBuf_Len;

				int nReturnCode = -1;
				nReturnCode = WSASendTo(m_socket,&wsaSendBuf,1,&BytesofSend,dFlag,(struct sockaddr*)&m_SendAddr,iLen,NULL,NULL);

				if (nReturnCode != 0)
				{
					//std::cout << "Send error";
					QMessageBox::critical(NULL, "Error", "Send Error");
					return false;
				}
				else{
					return true;
				}
			}
			else{
				QMessageBox::critical(NULL, "Error", "Client Init Failed");
				return false;
			}
		}
	};


	// UDP Server
	class CUdpServer{
	private:
		SOCKET m_socket;
		SOCKADDR_IN m_LocalAddr;
		SOCKADDR_IN m_RecvAddr;
		WSABUF m_WSARecvBuf;

		bool m_bConStus;
		bool m_bInitStus;
	public:
		CUdpServer()
		{
			// Status initialize
			m_bConStus = false;	
			m_bInitStus = false;

			// Initialize the buffer
			m_WSARecvBuf.len = 512;
			m_WSARecvBuf.buf = new char[512];

			// Socket initialize
			WSADATA wsaData;

			int err = WSAStartup(0x202, &wsaData);

			if (err != 0)
			{
				//std::cout << "WSAStartup failed with error: " << err;
				QMessageBox::critical(NULL, "Error", "WSAStartup Failed with Error");
				m_bConStus = false;
			}
			else{
				m_bConStus = true;
			}
		}
		~CUdpServer()
		{
			closesocket(m_socket);
		}
	public:
		bool Init(const char *IP, const unsigned short port)
		{
			if (m_bConStus == true)
			{
				// initialize socket
				m_socket = WSASocket(AF_INET,SOCK_DGRAM,IPPROTO_UDP,NULL,0,WSA_FLAG_OVERLAPPED);
				if (m_socket == INVALID_SOCKET)
				{
					//std::cout << "Socket create error" << std::endl;
					QMessageBox::critical(NULL, "Error", "Socket Create Error");
					return false;
				}

				// initialize address	
				m_LocalAddr.sin_addr.S_un.S_addr = INADDR_ANY;
				m_LocalAddr.sin_family = PF_INET;
				m_LocalAddr.sin_port = htons(port);

// 				// 
// 				m_RecvAddr.sin_addr.S_un.S_addr = inet_addr("172.16.13.155");
// 				m_RecvAddr.sin_family = PF_INET;
// 				m_RecvAddr.sin_port = htons(5500);

				int ret_bind = bind(m_socket, (struct sockaddr*)&m_LocalAddr, sizeof(m_LocalAddr));
				if (ret_bind != 0)
				{
					//std::cout << "Socket bind failed" << std::endl;
					QMessageBox::critical(NULL, "Error", "Socket Bind Failed");
					return false;
				}
				else{
					m_bInitStus = true;
					return true;
				}
			}
			else{
				//std::cout << "WSAStartup Initialize Failed, reinitialize first" << std::endl;
				QMessageBox::critical(NULL, "Error", "WSAStartup Initialize Failed, ReInitialize!");
				return false;
			}
		}

		bool CloseSock()
		{
			int ret = closesocket(m_socket);
			if (ret == 0)
			{
				return true;
			}
			// TODO(CJH): Add error lists, to judge which error occurs
			else{
				return false;
			}
		}

		bool Recv(char RecvBuf[], int RecvBuf_Len)
		{
			if (m_bInitStus == true)
			{
				DWORD BytesofRecvd = 0;
				DWORD dFlag = 0;
				int iLen = sizeof(m_RecvAddr);


//				int recv_byte = recvfrom(m_socket, RecvBuf, RecvBuf_Len, 0, (SOCKADDR *)&m_RecvAddr, &iLen);
				int ret_recv = WSARecvFrom(m_socket, &m_WSARecvBuf, 1, &BytesofRecvd, &dFlag, (struct sockaddr *)&m_RecvAddr, &iLen, NULL, NULL);
				int ret = WSAGetLastError();
				if (ret_recv != 0){
					//std::cout << "Recv Error!" << std::endl;
					QMessageBox::critical(NULL, "Error", "Receive Error!");
					memset(RecvBuf,0,RecvBuf_Len);
					return false;
				}
				else{
					for (int i = 0; i < 256; ++i)
					{
						RecvBuf[i] = m_WSARecvBuf.buf[i];
					}
					return true;
				}
			}
			else{
				//std::cout << "Socket Bind Failed, Please reBind" << std::endl;
				QMessageBox::critical(NULL, "Error", "Socket Bind Failed, ReBind!");
				return false;
			}
		}
	};


	// TCP Client
	class CTcpClient{
	private:
		bool m_bInit;
	public:
		CTcpClient()
		{
			m_bInit = true;	

			WSADATA wsaData;

			int err = WSAStartup(0x202, &wsaData);

			if (err != 0)
			{
				//std::cout << "WSAStartup failed with error: " << err;
				QMessageBox::critical(NULL, "Error", "WSAStartup Failed with Error!");
				m_bInit = true;
			}
		}
		~CTcpClient();


	};

	// TCP Server
	class CTcpServer{
	public:

	};
}

#endif



