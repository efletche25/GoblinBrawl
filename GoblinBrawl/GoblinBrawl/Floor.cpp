#include "stdafx.h"
#include "Floor.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"

Floor::Floor()  {}


Floor::~Floor() {}

bool Floor::Init( ModelLoader* modelLoader ) {
	modelLoader->Load( "floor.lxo" );
	mesh = modelLoader->GetMesh();
	if( !mesh ) {
		return false;
	}
	return true;
}

void Floor::Draw( ID3D11DeviceContext*	context ) {
	UINT stride = sizeof( Vertex::SimpleVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
}