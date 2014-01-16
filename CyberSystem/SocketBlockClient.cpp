// SocketBlockClient.cpp: implementation of the CSocketBlockClient class.
//
//////////////////////////////////////////////////////////////////////
//#define _AFXDLL

#include <afx.h>
#include "SocketBlockClient.h"
#include <QMessageBox>



#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSocketBlockClient::CSocketBlockClient()
{
	m_bInit=TRUE;
	m_hSocket=NULL;
	WSADATA wsaData;
	WORD version=MAKEWORD(2,2);
	int ret=WSAStartup(version,&wsaData);
	if(ret!=0)
	{
		TRACE("Initilize Error!\n");
		m_bInit=FALSE;
		QMessageBox::about(NULL, "About", "socket 初始化错误");
		//::MessageBox(NULL,"socket 初始化错误","提示信息",NULL);
	}
}

CSocketBlockClient::~CSocketBlockClient()
{
}




int CSocketBlockClient::ConnectServer(char *IP, UINT nPort)
{
	if(!m_bInit)
	{
		QMessageBox::about(NULL, "About", "winsock 初始化错误!");
		//::MessageBox(NULL,"winsock 初始化错误!","错误提示",NULL);
		return 1;
	}

	if(m_hSocket!=NULL)
	{
		closesocket(m_hSocket);
		m_hSocket = NULL;	
	}

	if(m_hSocket==NULL)
	{	
		m_hSocket=socket(AF_INET,SOCK_STREAM,0);
		ASSERT(m_hSocket!=NULL);
	}
	
	m_addr.sin_family = AF_INET;
	m_addr.sin_addr.S_un.S_addr = inet_addr(IP);
	m_addr.sin_port = htons(nPort);

	int ret=10;
	int error=0;
	ret=connect(m_hSocket,(LPSOCKADDR)&m_addr,sizeof(m_addr));
	
	if(ret==SOCKET_ERROR)
	{
		m_hSocket=NULL;
		return 1;
	}
	return 0;
}

int CSocketBlockClient::SendData(char *buff)
{
	if(!m_bInit)
	{
		QMessageBox::about(NULL, "About", "winsock 初始化错误!");
		//::MessageBox(NULL,"winsock 初始化错误!","错误提示",NULL);
		return 1;
	}

	char lastbuf[ORDER_BUF_LEN]={0};
	char headlen[5]={0};
	int end =0;
	char *dataBegin=NULL;
	int dataLen=strlen(buff);

	////////////////////////////
	itoa(dataLen,headlen,10);
	int totalLen=dataLen+5;
	int hbit=strlen(headlen);
	strcpy(lastbuf,headlen);

	while(hbit<5)
	{
		lastbuf[hbit]=' ';
		hbit++;
	}

//	strcat(lastbuf,buff);
	strncpy(lastbuf,buff,ORDER_BUF_LEN);

	dataBegin=lastbuf;

	for(int j=0;j<totalLen/ORDER_BUF_LEN;j++)
	{
		end=send(m_hSocket,dataBegin,ORDER_BUF_LEN,0); 
		if(end==SOCKET_ERROR)
		{
			return 1;
		}
		dataBegin+=ORDER_BUF_LEN;
	}
	if(totalLen%ORDER_BUF_LEN)
	{
		end=send(m_hSocket,dataBegin,totalLen%ORDER_BUF_LEN,0); 
		if(end==SOCKET_ERROR)
		{
			return 1;
		}
	}
	
	return 0;

}

int CSocketBlockClient::SendData2(char *buff)
{
	if(!m_bInit)
	{
		QMessageBox::about(NULL, "About", "winsock 初始化错误!");
		//::MessageBox(NULL,"winsock 初始化错误!","错误提示",NULL);
		return 1;
	}

	char lastbuf[ORDER_BUF_LEN2]={0};
	char headlen[5]={0};
	int end =0;
	char *dataBegin=NULL;
	int dataLen=strlen(buff);

	////////////////////////////
	itoa(dataLen,headlen,10);
	int totalLen=dataLen+5;
	int hbit=strlen(headlen);
	strcpy(lastbuf,headlen);

	while(hbit<5)
	{
		lastbuf[hbit]=' ';
		hbit++;
	}

	strcat(lastbuf,buff);

	dataBegin=lastbuf;

	for(int j=0;j<totalLen/ORDER_BUF_LEN2;j++)
	{
		end=send(m_hSocket,dataBegin,ORDER_BUF_LEN2,0); 
		if(end==SOCKET_ERROR)
		{
			return 1;
		}
		dataBegin+=ORDER_BUF_LEN2;
	}
	if(totalLen%ORDER_BUF_LEN2)
	{
		end=send(m_hSocket,dataBegin,totalLen%ORDER_BUF_LEN2,0); 
		if(end==SOCKET_ERROR)
		{
			return 1;
		}
	}

	return 0;

}

int CSocketBlockClient::ReceiveData(char *buff)
{
	if(!m_bInit)
	{
		QMessageBox::about(NULL, "About", "winsock 初始化错误!");
		//::MessageBox(NULL,"winsock 初始化错误!","错误提示",NULL);
		return 1;
	}
	CString msg;
	char headLen[6]={0};
	char tmp[SENSE_BUF_LEN+1]={0};
	char* p=buff;
//	int nByteThisTime;
	int nByteRev = 0;

	int ret=recv(m_hSocket,tmp,FIRST_REV_BYTES,0);
	
	
	strncpy(buff,&tmp[0],FIRST_REV_BYTES);
	/*nByteThisTime = ret;
	nByteRev += nByteThisTime;
*/
	if(ret==SOCKET_ERROR)
	{
		int result=GetLastError();
		if(result==WSAECONNRESET)
		{
			return WSAECONNRESET;
		}
	}

/*	strncpy(headLen,tmp,5);
	int dataLen=atoi(headLen);
	int totalLen=dataLen+5;

	strcpy(buff,&tmp[5]);

	while(nByteRev<totalLen)
	{
		for(int j=0;j<(totalLen-nByteRev)/SENSE_BUF_LEN;j++)
		{
			ret=recv(m_hSocket,tmp,SENSE_BUF_LEN,0);
			
			if(ret==SOCKET_ERROR)
			{
				int result=GetLastError();
				if(result==WSAECONNRESET)
				{
					return WSAECONNRESET;
				}
			}

			nByteThisTime = ret;
			strncat(buff,&tmp[0],nByteThisTime);
			nByteRev += nByteThisTime;
		}
		if((totalLen-nByteRev)%SENSE_BUF_LEN)
		{
			ret=recv(m_hSocket,tmp,(totalLen-nByteRev)%SENSE_BUF_LEN,0);
			
			if(ret==SOCKET_ERROR)
			{
				int result=GetLastError();
				if(result==WSAECONNRESET)
				{
					return WSAECONNRESET;
				}
			}
			
			nByteThisTime = ret;
			strncat(buff,&tmp[0],nByteThisTime);
			nByteRev += nByteThisTime;
		}
	}
*/
	return 0;
}

int CSocketBlockClient::ReceiveData2(char *buff)
{
	if(!m_bInit)
	{
		QMessageBox::about(NULL, "About", "winsock 初始化错误!");
		//::MessageBox(NULL,"winsock 初始化错误!","错误提示",NULL);
		return 1;
	}
	CString msg;
	char headLen[6]={0};
	char tmp[SENSE_BUF_LEN2+1]={0};
	char* p=buff;
	int nByteThisTime;
	int nByteRev = 0;

	int ret=recv(m_hSocket,tmp,300,0);
	nByteThisTime = ret;
	nByteRev += nByteThisTime;

	if(ret==SOCKET_ERROR)
	{
		int result=GetLastError();
		if(result==WSAECONNRESET)
		{
			return WSAECONNRESET;
		}
	}

	strncpy(headLen,tmp,5);
	int dataLen=atoi(headLen);
	int totalLen=dataLen+5;

	strcpy(buff,&tmp[5]);

	while(nByteRev<totalLen)
	{
		for(int j=0;j<(totalLen-nByteRev)/SENSE_BUF_LEN2;j++)
		{
			ret=recv(m_hSocket,tmp,SENSE_BUF_LEN2,0);

			if(ret==SOCKET_ERROR)
			{
				int result=GetLastError();
				if(result==WSAECONNRESET)
				{
					return WSAECONNRESET;
				}
			}

			nByteThisTime = ret;
			strncat(buff,&tmp[0],nByteThisTime);
			nByteRev += nByteThisTime;
		}
		if((totalLen-nByteRev)%SENSE_BUF_LEN2)
		{
			ret=recv(m_hSocket,tmp,(totalLen-nByteRev)%SENSE_BUF_LEN2,0);

			if(ret==SOCKET_ERROR)
			{
				int result=GetLastError();
				if(result==WSAECONNRESET)
				{
					return WSAECONNRESET;
				}
			}

			nByteThisTime = ret;
			strncat(buff,&tmp[0],nByteThisTime);
			nByteRev += nByteThisTime;
		}
	}

	return 0;


}

void CSocketBlockClient::GetError(DWORD error)
{
	char strError[256]={0};

	switch(error)
	{
	case WSANOTINITIALISED:
		strcpy(strError,"初始化错误");
		break;
	case WSAENOTCONN:
		strcpy(strError,"对方没有启动");
		break;
	case WSAEWOULDBLOCK :
		strcpy(strError,"对方已经关闭");
		break;
	case WSAECONNREFUSED:
		strcpy(strError,"连接的尝试被拒绝");
		break;
	case WSAENOTSOCK:
		strcpy(strError,"在一个非套接字上尝试了一个操作");
		break;
	case WSAEADDRINUSE:
		strcpy(strError,"特定的地址已在使用中");
		break;
	case WSAECONNRESET:
		strcpy(strError,"与主机的连接被关闭");
		break;

	case 3:
		strcpy(strError,"套接字非空，检查socket是否已连接成功?");
		break;

	default:
		strcpy(strError,"哈哈，低级错误"); 
	}
	
	//AfxMessageBox(strError);
}


