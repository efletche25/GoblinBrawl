#include "stdafx.h"
#include "Floor.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "d3dUtil.h"
#include "d3dx11effect.h"
#include "Effects.h"

Floor::Floor() {}

Floor::~Floor() {}

bool Floor::Init( ModelLoader* modelLoader, ID3D11Device* device ) {
	modelLoader->Load( "floor.lxo" );
	mesh = modelLoader->GetMesh();
	if( !mesh ) {
		return false;
	}
	return true;
}

void XM_CALLCONV Floor::Draw( FXMMATRIX viewProj, ID3D11DeviceContext* context ) {

	context->IASetInputLayout( InputLayouts::Simple );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::SimpleVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	//The floor is always at 0 so worldViewProj == viewProj
	//XMMATRIX world = XMMatrixIdentity();
	//XMMATRIX worldViewProj = world*viewProj;
	//fxWorldViewProj->SetMatrix( (float*)&worldViewProj );
	Effects::SimpleFX->SetWorldWiewProj( viewProj );
	ID3DX11EffectTechnique* tech = Effects::SimpleFX->simpleTechnique;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}