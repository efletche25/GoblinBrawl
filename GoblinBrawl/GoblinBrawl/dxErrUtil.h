#pragma once

void dxErrorCheck( HRESULT hr );

#if defined(DEBUG) | defined(_DEBUG)
#ifndef HR
#define HR(x) dxErrorCheck(x)
#endif

#else
#ifndef HR
#define HR(x) (x)
#endif
#endif 

#define ReleaseCOM(x) { if(x){ x->Release(); x = 0; } }

#define SafeDelete(x) { delete x; x = 0; }