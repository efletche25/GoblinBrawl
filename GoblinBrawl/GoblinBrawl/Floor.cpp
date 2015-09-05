#include "stdafx.h"
#include "Floor.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "Effects.h"
#include "MathUtils.h"

Floor::Floor() :
mesh( nullptr ),
diffuseView( nullptr )
{}

Floor::~Floor() {}

bool Floor::Init( ModelLoader* modelLoader, ID3D11Device* device ) {
	modelLoader->Load( "floor.lxo" , Vertex::TERRAIN);
	mesh = modelLoader->GetMesh();
	if( !mesh ) {
		return false;
	}
	HR( D3DX11CreateShaderResourceViewFromFile( device, L"./art/textures/floor_color.tif", NULL, NULL, &diffuseView, NULL ) );
	mat.Ambient = XMFLOAT4( 0.02f, 0.3f, 0.5f, 1.0f );
	mat.Diffuse = XMFLOAT4( 0.8f, 0.8f, 0.8f, 1.0f );
	mat.Specular = XMFLOAT4( 0.3f, 0.3f, 0.3f, 32.0f );
	return true;
}

void XM_CALLCONV Floor::Draw( FXMMATRIX viewProj, FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context ) {
	context->IASetInputLayout( InputLayouts::Terrain );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::TerrainVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	XMMATRIX world = XMMatrixIdentity();
	XMMATRIX worldInvTranspose = MathUtils::InverseTranspose( world );
	XMMATRIX worldViewProj = world*viewProj;

	Effects::StaticGeomFX->SetWorld( world );
	Effects::StaticGeomFX->SetWorldInvTranspose( worldInvTranspose );
	Effects::StaticGeomFX->SetWorldViewProj( worldViewProj );
	Effects::StaticGeomFX->SetDiffuseMap( diffuseView );
	Effects::StaticGeomFX->SetEyePosW( cameraPos );
	Effects::StaticGeomFX->SetPointLights( pointLights.data() );
	Effects::StaticGeomFX->SetMaterial( mat );

	ID3DX11EffectTechnique* tech = Effects::StaticGeomFX->staticGeomLight5Tech;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}