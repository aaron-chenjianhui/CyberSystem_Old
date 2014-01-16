

#ifndef AFX_SOCKETBLOCKCLIENT_H__708C81C0_5743_40EF_ABC5_30CBE4703C8C__INCLUDED_
#define AFX_SOCKETBLOCKCLIENT_H__708C81C0_5743_40EF_ABC5_30CBE4703C8C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include <winsock2.h>
#include "SocketDefine.h"


class CSocketBlockClient  
{
public:
	CSocketBlockClient();
	virtual ~CSocketBlockClient();
public:

	int ReceiveData(char* buff);
	int ReceiveData2(char* buff);
	int SendData(char* buff);	
	int SendData2(char* buff);	
	int ConnectServer(char* IP,UINT nPort);		
	void GetError(DWORD error);
public:
	SOCKET m_hSocket;							
	sockaddr_in m_addr;							
	UINT m_uPort;												
	BOOL m_bInit;								


};

#endif // !defined(AFX_SOCKETBLOCKCLIENT_H__708C81C0_5743_40EF_ABC5_30CBE4703C8C__INCLUDED_)

