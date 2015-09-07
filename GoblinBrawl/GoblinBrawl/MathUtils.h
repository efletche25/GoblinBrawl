#pragma once
#include "DirectX_11_1_Includes.h"

using namespace DirectX;

class MathUtils {
public:
	static XMMATRIX InverseTranspose( CXMMATRIX M ) {
		// Inverse-transpose is just applied to normals.  So zero out 
		// translation row so that it doesn't get into our inverse-transpose
		// calculation--we don't want the inverse-transpose of the translation.
		XMMATRIX A = M;
		A.r[3] = XMVectorSet( 0.0f, 0.0f, 0.0f, 1.0f );

		XMVECTOR det = XMMatrixDeterminant( A );
		return XMMatrixTranspose( XMMatrixInverse( &det, A ) );
	}
};

