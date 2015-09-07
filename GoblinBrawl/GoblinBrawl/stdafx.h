// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once
#if defined(DEBUG) | defined(_DEBUG)
#define _CRT_SECURE_NO_WARNINGS
#undef NDDEBUG
#else
#define NDEBUG
#endif
#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

// C RunTime Header Files
#include <stdlib.h>

#if defined(DEBUG) | defined(_DEBUG)
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

#if defined(_XBOX_ONE) && defined(_TITLE)
#include <d3d11_x.h>
#else
#include <d3d11_1.h>
#endif

#include "dxErrUtil.h"

// TODO: reference additional headers your program requires here
