#pragma once
#include<D3DX11.h>
#include<dxErr.h>

#if defined(DEBUG) || defined(_DEBUG)
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif

// Simple error checker
#if defined(DEBUG) | defined(_DEBUG)
#ifndef HR
#define HR(x)													\
		{														\
		HRESULT hr = (x);										\
		if(FAILED(hr))											\
				{												\
			DXTrace(__FILE__, (DWORD)__LINE__, hr, L#x, true);	\
				}												\
		}
#endif

#else
#ifndef HR
#define HR(x) (x)
#endif
#endif 

#define ReleaseCOM(x) { if(x){ x->Release(); x = 0; } }

#define SafeDelete(x) { delete x; x = 0; }