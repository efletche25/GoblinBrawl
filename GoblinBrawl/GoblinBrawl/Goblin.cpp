#include "stdafx.h"
#include "Goblin.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "MyEffects.h"
#include "MathUtils.h"
#include "Skeleton.h"
#include "WICTextureLoader.h"

Goblin::Goblin() :
mesh( nullptr ),
diffuseView( nullptr ) {}

Goblin::~Goblin() {
	delete skeleton;
	delete mesh;
}

bool Goblin::Init( ModelLoader* modelLoader, ID3D11Device* device, Keyboard::KeyboardStateTracker* kb, PLAYER player ) {
	modelLoader->Load( "Goblin2.fbx", Vertex::CHARACTER_SKINNED );
	mesh = modelLoader->GetMesh();
	if( mesh->VB()==nullptr ) {
		return false;
	}
	skeleton = modelLoader->GetSkeleton();
	if( skeleton==nullptr ) {
		return false;
	}
	HR( CreateWICTextureFromFile( device, L"./art/textures/goblin_color.tif", NULL, &diffuseView, NULL ) );
	mat.Ambient = XMFLOAT4( 0.02f, 0.3f, 0.5f, 1.0f );
	mat.Diffuse = XMFLOAT4( 0.8f, 0.8f, 0.8f, 1.0f );
	mat.Specular = XMFLOAT4( 0.3f, 0.3f, 0.3f, 32.0f );

	pos = XMMatrixIdentity();
	rot = XMMatrixIdentity();
	//scale = XMMatrixIdentity();
	scale = XMMatrixScaling( 0.01f, 0.01f, 0.01f );

	this->kb = kb;
	this->player = player;

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

	MyEffects::CharacterSkinnedFX->SetWorld( world );
	MyEffects::CharacterSkinnedFX->SetWorldInvTranspose( worldInvTranspose );
	MyEffects::CharacterSkinnedFX->SetWorldViewProj( worldViewProj );
	MyEffects::CharacterSkinnedFX->SetDiffuseMap( diffuseView );
	MyEffects::CharacterSkinnedFX->SetEyePosW( cameraPos );
	MyEffects::CharacterSkinnedFX->SetPointLights( pointLights.data() );
	MyEffects::CharacterSkinnedFX->SetMaterial( mat );
	auto a = skeleton->GetFinalTransforms();
	auto b = skeleton->BoneCount();
	MyEffects::CharacterSkinnedFX->SetBoneTransforms( a, b );

	ID3DX11EffectTechnique* tech = MyEffects::CharacterSkinnedFX->characterSkinnedLight5Tech;
	D3DX11_TECHNIQUE_DESC td;
	tech->GetDesc( &td );
	for( UINT p = 0; p<td.Passes; ++p ) {
		tech->GetPassByIndex( p )->Apply( 0, context );
		context->DrawIndexed( mesh->IndexCount(), 0, 0 );
	}
}

void Goblin::Update( float dt ) {
	UpdateActions();
	DebugActionDisplay();
	//XMVECTOR translate = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	//XMVECTOR rotQuat = XMLoadFloat4( &XMFLOAT4( 0.f, 0.f, 0.f, 1.f ) );
	//XMVECTOR scale = XMLoadFloat4( &XMFLOAT4( 1.f, 1.f, 1.f, 1.f ) );

	XMVECTOR rotQuat = XMQuaternionRotationRollPitchYaw( 0.f, -XM_PIDIV2/2, 0.f );
	//skeleton->UpdateTransformByName( translate, rotQuat, scale, "Skeleton_Upper_Spine" );

	rotQuat = XMQuaternionRotationRollPitchYaw( 0.f, XM_PIDIV2/2, 0.f );
	//skeleton->UpdateTransformByName( translate, rotQuat, scale, "Skeleton_Neck" );

	rotQuat = XMQuaternionRotationRollPitchYaw( -XM_PIDIV2, 0.f, 0.f );
	//skeleton->UpdateTransformByName( translate, rotQuat, scale, "Skeleton_Elbow_L" );
	Bone* elbow = skeleton->GetBoneByName( "Skeleton_Lower_Spine" );
	XMMATRIX rot = XMMatrixRotationZ( XM_PI/1800.f );
	XMMATRIX elbowTransform = elbow->localTransform;
	XMMATRIX bentElbow = elbowTransform*rot;
	elbow->localTransform = bentElbow;
}

void XM_CALLCONV Goblin::SetPos( FXMVECTOR _pos ) {
	pos = XMMatrixTranslationFromVector( _pos );
}

FXMVECTOR XM_CALLCONV Goblin::getPos() {
	XMVECTOR outScale;
	XMVECTOR outRotQuat;
	XMVECTOR outTrans;
	XMMatrixDecompose( &outScale, &outRotQuat, &outTrans, pos );
	return outTrans;
}

void XM_CALLCONV Goblin::SetRot( FXMVECTOR _pos ) {
	rot = XMMatrixRotationRollPitchYawFromVector( _pos );
}

void Goblin::UpdateActions() {
	if( player==PLAYER_1 ) {
		// Player 1 keys
		action.Forward = kb->lastState.W;
		action.Back = kb->lastState.S;
		action.Left = kb->lastState.A;
		action.Right = kb->lastState.D;
		action.Attack = kb->lastState.V;
		action.Jump = kb->lastState.B;
		action.Duck = kb->lastState.N;
	} else {
		// Player 2 keys
		action.Forward = kb->lastState.Up;
		action.Back = kb->lastState.Down;
		action.Left = kb->lastState.Left;
		action.Right = kb->lastState.Right;
		action.Attack = kb->lastState.NumPad0;
		action.Jump = kb->lastState.NumPad2;
		action.Duck = kb->lastState.Decimal;
	}
}

void Goblin::ResetActions() {
	action.Forward = false;
	action.Back = false;
	action.Left = false;
	action.Right = false;
	action.Attack = false;
	action.Jump = false;
	action.Duck = false;
}

void Goblin::DebugActionDisplay() {
	fprintf( stdout, "Forward:%i\nBack:%i\nRight:%i\nLeft:%i\nAttack:%i\nJump:%i\nDuck:%i\n",
		action.Forward,
		action.Back,
		action.Left,
		action.Right,
		action.Attack,
		action.Jump,
		action.Duck );
}