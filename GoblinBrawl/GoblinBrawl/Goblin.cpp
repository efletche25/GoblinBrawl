#include "stdafx.h"
#include "Goblin.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "d3dUtil.h"
#include "d3dx11effect.h"
#include "Effects.h"
#include "MathUtils.h"
#include "Skeleton.h"

Goblin::Goblin() :
mesh(nullptr),
diffuseView(nullptr)
{}

Goblin::~Goblin() {
	delete skeleton;
	delete mesh;
}

bool Goblin::Init( ModelLoader* modelLoader, ID3D11Device* device ) {
	modelLoader->Load( "Goblin_Anim.fbx", Vertex::CHARACTER_SKINNED );
	//modelLoader->Load( "goblin.lxo", Vertex::CHARACTER );
	mesh = modelLoader->GetMesh();
	if( mesh->VB()==nullptr ) {
		return false;
	}
	skeleton = modelLoader->GetSkeleton();
	if( skeleton==nullptr ) {
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
	context->IASetInputLayout( InputLayouts::CharacterSkinned );
	context->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
	UINT stride = sizeof( Vertex::CharacterSkinnedVertex );
	UINT offset = 0;
	ID3D11Buffer* buffers[1] = { mesh->VB() };
	context->IASetVertexBuffers( 0, 1, &buffers[0], &stride, &offset );
	context->IASetIndexBuffer( mesh->IB(), mesh->IndexFormat(), 0 );

	XMMATRIX world = scale * rot * pos;
	XMMATRIX worldInvTranspose = MathUtils::InverseTranspose( world );
	XMMATRIX worldViewProj = world*viewProj;

	Effects::CharacterSkinnedFX->SetWorld( world );
	Effects::CharacterSkinnedFX->SetWorldInvTranspose( worldInvTranspose );
	Effects::CharacterSkinnedFX->SetWorldViewProj( worldViewProj );
	Effects::CharacterSkinnedFX->SetDiffuseMap( diffuseView );
	Effects::CharacterSkinnedFX->SetEyePosW( cameraPos );
	Effects::CharacterSkinnedFX->SetPointLights( pointLights.data() );
	Effects::CharacterSkinnedFX->SetMaterial( mat );
	auto a = skeleton->GetFinalTransforms();
	auto b = skeleton->BoneCount();
	Effects::CharacterSkinnedFX->SetBoneTransforms( a, b );

	ID3DX11EffectTechnique* tech = Effects::CharacterSkinnedFX->characterSkinnedLight5Tech;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}

void Goblin::Update( float dt ) {
	XMVECTOR translate = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	XMVECTOR rotQuat = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	XMVECTOR scale = XMLoadFloat4( &XMFLOAT4( 1.1f, 1.1f, 2.1f, 1.f ) );
	skeleton->UpdateTransformByName( translate, rotQuat, scale, "Skeleton_Clavicle_R" );
}

void XM_CALLCONV Goblin::SetPos( FXMVECTOR _pos ) {
	pos = XMMatrixTranslationFromVector( _pos );
}

void XM_CALLCONV Goblin::SetRot( FXMVECTOR _pos ) {
	rot = XMMatrixRotationRollPitchYawFromVector( _pos );
}