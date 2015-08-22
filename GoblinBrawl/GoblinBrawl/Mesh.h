#pragma once
#include "D3D11.h"
class Mesh {
public:
	Mesh();
	~Mesh();
	inline ID3D11Buffer * IB() {
		return ib;
	}
	inline ID3D11Buffer * VB() {
		return vb;
	}
	inline void SetIB( ID3D11Buffer* ib ) {
		this->ib = ib;
	}
	inline void SetVB( ID3D11Buffer* vb ) {
		this->vb = vb;
	}
private:
	ID3D11Buffer*	vb;
	ID3D11Buffer*	ib;
	DXGI_FORMAT		indexBufferFormat;
	UINT			vertexStride;
};

