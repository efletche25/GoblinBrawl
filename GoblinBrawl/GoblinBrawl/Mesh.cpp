#include "stdafx.h"
#include "Mesh.h"
#include "d3dUtil.h"

Mesh::Mesh() : 
vb(0),
ib(0),
indexBufferFormat(DXGI_FORMAT_R16_UINT),
vertexStride(0)
{}

Mesh::~Mesh() {
	ReleaseCOM( vb );
	ReleaseCOM( ib );
}
