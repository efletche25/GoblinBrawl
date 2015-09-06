#include "stdafx.h"
#include "dxErrUtil.h"
#include <strsafe.h>

void dxErrorCheck( HRESULT hr ) {
	LPWSTR lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();
	if( FAILED( hr ) ) {
		FormatMessage( FORMAT_MESSAGE_FROM_SYSTEM|
			FORMAT_MESSAGE_ALLOCATE_BUFFER|
			FORMAT_MESSAGE_IGNORE_INSERTS, NULL, hr,
			MAKELANGID( LANG_NEUTRAL, SUBLANG_DEFAULT ),
			(LPTSTR)&lpMsgBuf, 0, NULL );
		lpDisplayBuf = (LPVOID)LocalAlloc( LMEM_ZEROINIT,
			(lstrlen( lpMsgBuf ) * sizeof( TCHAR )) );
		StringCchPrintf( (LPTSTR)lpDisplayBuf, LocalSize( lpDisplayBuf )/sizeof( TCHAR ), TEXT( "Failed with error %d: %s" ), dw, lpMsgBuf );
		MessageBox( NULL, (LPCTSTR)lpDisplayBuf, TEXT( "Error" ), MB_OK );

		LocalFree( lpMsgBuf );
		LocalFree( lpDisplayBuf );
		ExitProcess( dw );
	}

}