#include "stdafx.h"
#include "Goblin.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "d3dUtil.h"
#include "d3dx11effect.h"
#include "Effects.h"
#include "MathUtils.h"

Goblin::Goblin() :
mesh(nullptr),
diffuseView(nullptr)
{}

Goblin::~Goblin() {}

bool Goblin::Init( ModelLoader* modelLoader, ID3D11Device* device ) {
	modelLoader->Load( "Goblin_Anim.fbx", Vertex::CHARACTER );
	//modelLoader->Load( "goblin.lxo", Vertex::CHARACTER );
	mesh = modelLoader->GetMesh();
	if( !mesh ) {
		return false;
	}
	HR( D3DX11CreateShaderResourceViewFromFile( device, L"./art/textures/goblin_color.tif", NULL, NULL, &diffuseView, NULL ) );
	mat.Ambient = XMFLOAT4( 0.02f, 0.3f, 0.5f, 1.0f );
	mat.Diffuse = XMFLOAT4( 0.8f, 0.8f, 0.8f, 1.0f );
	mat.Specular = XMFLOAT4( 0.3f, 0.3f, 0.3f, 32.0f );

	pos = XMMatrixIdentity();
	rot = XMMatrixIdentity();
	//scale = XMMatrixIdentity();
	scale = XMMatrixScaling( 0.01f, 0.01f, 0.01f );

	return true;
}

void XM_CALLCONV Goblin::Draw( FXMMATRIX viewProj, FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context ) {
	context->IASetInputLayout( InputLayouts::Character );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::CharacterVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	XMMATRIX world = scale * rot * pos;
	XMMATRIX worldInvTranspose = MathUtils::InverseTranspose( world );
	XMMATRIX worldViewProj = world*viewProj;

	Effects::CharacterFX->SetWorld( world );
	Effects::CharacterFX->SetWorldInvTranspose( worldInvTranspose );
	Effects::CharacterFX->SetWorldViewProj( worldViewProj );
	Effects::CharacterFX->SetDiffuseMap( diffuseView );
	Effects::CharacterFX->SetEyePosW( cameraPos );
	Effects::CharacterFX->SetPointLights( pointLights.data() );
	Effects::CharacterFX->SetMaterial( mat );

	ID3DX11EffectTechnique* tech = Effects::CharacterFX->characterLight5Tech;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}

void XM_CALLCONV Goblin::SetPos( FXMVECTOR _pos ) {
	pos = XMMatrixTranslationFromVector( _pos );
}

void XM_CALLCONV Goblin::SetRot( FXMVECTOR _pos ) {
	rot = XMMatrixRotationRollPitchYawFromVector( _pos );
}