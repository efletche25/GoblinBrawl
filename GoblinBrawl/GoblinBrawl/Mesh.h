#pragma once
#include "DirectX_11_1_Includes.h"
class Mesh {
public:
	Mesh();
	~Mesh();
	inline ID3D11Buffer* IB() {
		return ib;
	}
	inline ID3D11Buffer* VB() {
		return vb;
	}
	inline UINT IndexCount() {
		return indexCount;
	}
	inline DXGI_FORMAT IndexFormat() {
		return indexFormat;
	}
	inline void SetIB( ID3D11Buffer* ib, UINT indexCount ) {
		this->ib = ib;
		this->indexCount = indexCount;
	}
	inline void SetVB( ID3D11Buffer* vb ) {
		this->vb = vb;
	}
private:
	ID3D11Buffer*	vb;
	ID3D11Buffer*	ib;
	UINT			indexCount;
	DXGI_FORMAT		indexFormat;
	UINT			vertexStride;
};

