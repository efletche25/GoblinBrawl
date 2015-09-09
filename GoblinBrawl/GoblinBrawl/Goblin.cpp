#include "stdafx.h"
#include "Goblin.h"
#include "ModelLoader.h"
#include "Vertex.h"
#include "Mesh.h"
#include "MyEffects.h"
#include "MathUtils.h"
#include "Skeleton.h"
#include "WICTextureLoader.h"
#include "PhysicsWorld.h"
#include "Bullet/BulletDynamics/Character/btKinematicCharacterController.h"
#include "Bullet/BulletCollision/CollisionDispatch/btGhostObject.h"

Goblin::Goblin() :
mesh( nullptr ),
diffuseView( nullptr ),
ghostObject(nullptr),
controller(nullptr),
forwardAmount(0),
turnAmount(0),
strafeAmount(0),
forwardSpeed(1.f),
turnSpeed(2.f),
strafeSpeed(0.7f)
{}

Goblin::~Goblin() {
	delete skeleton;
	delete mesh;
}

bool Goblin::Init( ModelLoader* modelLoader, ID3D11Device* device, Keyboard::KeyboardStateTracker* kb, GamePad* gamePad, PLAYER player, PhysicsWorld* physicsWorld ) {
	// Model
	modelLoader->Load( "Goblin2.fbx", Vertex::CHARACTER_SKINNED );
	mesh = modelLoader->GetMesh();
	if( mesh->VB()==nullptr ) {
		return false;
	}
	skeleton = modelLoader->GetSkeleton();
	if( skeleton==nullptr ) {
		return false;
	}

	//Texture
	HR( CreateWICTextureFromFile( device, L"./art/textures/goblin_color.tif", NULL, &diffuseView, NULL ) );
	mat.Ambient = XMFLOAT4( 0.02f, 0.3f, 0.5f, 1.0f );
	mat.Diffuse = XMFLOAT4( 0.8f, 0.8f, 0.8f, 1.0f );
	mat.Specular = XMFLOAT4( 0.3f, 0.3f, 0.3f, 32.0f );

	// Start Position
	XMFLOAT4 goblinPos = XMFLOAT4( 0.f, 2.3f, 0.f, 1.f );
	XMVECTOR xmVectorPos = XMLoadFloat4( &goblinPos );
	if( player==PLAYER_1 ) {
		xmVectorPos = XMVectorSet( 0.f, 2.3f, 0.f, 1.0f );
	} else {
		xmVectorPos = XMVectorSet( 20.f, 5.f, 10.f, 1.0f );
	}
	SetPos( xmVectorPos );
	rot = XMMatrixIdentity();
	//scale = XMMatrixIdentity();
	scale = XMMatrixScaling( 0.01f, 0.01f, 0.01f );

	// Keyboard Controller
	this->kb = kb;
	this->player = player;

	// GamePad
	this->gamePad = gamePad;

	// Physics
	this->physicsWorld = physicsWorld;
	btScalar controllerWidth = 0.4;
	btScalar controllerHeight = 1.5;
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin( btVector3( goblinPos.x, goblinPos.y+10, goblinPos.z ) );
	ghostObject = new btPairCachingGhostObject();
	ghostObject->setWorldTransform( startTransform );
	physicsWorld->getPairCache()->getOverlappingPairCache()->setInternalGhostPairCallback( new btGhostPairCallback() );
	btConvexShape* capsule = new btCapsuleShape( controllerWidth, controllerHeight );
	physicsWorld->AddCollisionShape( capsule );
	ghostObject->setCollisionShape( capsule );
	ghostObject->setCollisionFlags( btCollisionObject::CF_CHARACTER_OBJECT );
	btScalar stepHeight = btScalar( 0.35 );
	controller = new btKinematicCharacterController( ghostObject, capsule, stepHeight );
	physicsWorld->World()->addCollisionObject( ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter );
	physicsWorld->World()->addAction( controller );

	// Finite State Machine
	InitFSM();

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
	fprintf( stdout, "DT : %f", dt );
	UpdateActions();
	DebugActionDisplay();
	fsm->Update(dt);
	UpdateController(dt);
	UpdateModelTransforms();
}

void Goblin::UpdateController(float dt) {
	btTransform controllerTransform;
	controllerTransform = ghostObject->getWorldTransform();
	
	btVector3 forwardDir = controllerTransform.getBasis()[2];
	fprintf( stdout, "forwardDir=%f,%f,%f\n", forwardDir[0], forwardDir[1], forwardDir[2] );
	btVector3 upDir = controllerTransform.getBasis()[1];
	btVector3 strafeDir = controllerTransform.getBasis()[0];
	forwardDir.normalize();
	upDir.normalize();
	strafeDir.normalize();

	btScalar walkSpeed =  btScalar( forwardSpeed ) * forwardAmount * dt;
	fprintf( stdout, "walkSpeed=%f\n", walkSpeed );

	if( turnAmount!=0.f ) {
		btMatrix3x3 orn = ghostObject->getWorldTransform().getBasis();
		btQuaternion rotQuat = btQuaternion( btVector3( 0, 1, 0 ), -turnSpeed*turnAmount*dt ); // negative to convert right handed to left handed
		rotQuat.normalize();
		orn *= btMatrix3x3( rotQuat );
		ghostObject->getWorldTransform().setBasis( orn );
	}
	forwardDir *= btVector3( 1.0, 1.0, -1.0 ); // Switch Right Handed to left handed
	controller->setWalkDirection( forwardDir*walkSpeed );
}

void Goblin::UpdateModelTransforms() {
	btTransform controllerTransform = ghostObject->getWorldTransform();
	btVector3 btPos = controllerTransform.getOrigin();
	XMVECTOR dxPos = XMLoadFloat4( &XMFLOAT4( btPos.x(), btPos.y(), btPos.z(), 1.f ) );
	SetPos( dxPos );
	btMatrix3x3 btRot = controllerTransform.getBasis().transpose();
	XMMATRIX dxMat = XMMATRIX(
		btRot[0].x(), btRot[0].y(), btRot[0].z(), 0,
		btRot[1].x(), btRot[1].y(), btRot[1].z(), 0,
		btRot[2].x(), btRot[2].y(), btRot[2].z(), 0,
		0, 0, 0, 1.f );
	rot = dxMat;
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

void XM_CALLCONV Goblin::SetRot( FXMVECTOR _rot ) {
	rot = XMMatrixRotationRollPitchYawFromVector( _rot );
}

void Goblin::UpdateActions() {
	ResetActions();
	auto gpState = gamePad->GetState( player ); // PLAYER_1 == gamepad 0
	if( gpState.IsConnected() ) {
		strafeAmount = gpState.thumbSticks.leftX;
		forwardAmount = gpState.thumbSticks.leftY;
		turnAmount = -gpState.thumbSticks.rightX;
		if( forwardAmount>0 ) {
			action.Forward = true;
		} else if( forwardAmount<0 ) {
			action.Back = true;
		}
		if( turnAmount<0||strafeAmount<0 ) {
			action.Left = true;
		} else if( turnAmount>0||strafeAmount>0 ) {
			action.Right = true;
		}
		action.Attack = gpState.IsBPressed();
		action.Jump = gpState.IsAPressed();
		action.Duck = gpState.IsXPressed();
	} else {
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

		// This turn amount and forward amount will be overridden if using gamepad
		if( action.Forward ) {
			forwardAmount = 1.f;
		} else if( action.Back ) {
			forwardAmount = -1.f;
		}

		if( action.Left ) {
			turnAmount = -1.f;
		} else if( action.Right ) {
			turnAmount = 1.f;
		}
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
	fprintf( stdout, "\n\nForward:%i\nBack:%i\nRight:%i\nLeft:%i\nAttack:%i\nJump:%i\nDuck:%i\n",
		action.Forward,
		action.Back,
		action.Left,
		action.Right,
		action.Attack,
		action.Jump,
		action.Duck );
}

void Goblin::InitFSM() {
	
	fsm = new FSM<Goblin>( this );
	FSM<Goblin>::StateData idleStateData;
	idleStateData.Before = &Goblin::Idle_Before;
	idleStateData.Update = &Goblin::Idle_Update;
	idleStateData.After = &Goblin::Idle_After;
	fsm->AddState( FSM_STATE::IDLE, idleStateData );

	FSM<Goblin>::StateData forwardStateData;
	forwardStateData.Before = &Goblin::Forward_Before;
	forwardStateData.Update = &Goblin::Forward_Update;
	forwardStateData.After = &Goblin::Forward_After;
	fsm->AddState( FSM_STATE::FORWARD, forwardStateData );
	
	fsm->ChangeState( FSM_STATE::IDLE );
	fsm->Update( 42 );
	fsm->Update( 930 );
	fsm->Update( 930 );
	fsm->Update( 930 );
	fsm->ChangeState( FSM_STATE::FORWARD );
	fsm->Update( 42 );
	fsm->Update( 930 );
	fsm->ChangeState( FSM_STATE::IDLE );
	fsm->ChangeState( FSM_STATE::FORWARD );
	fsm->ChangeState( FSM_STATE::FORWARD );
	fsm->ChangeState( FSM_STATE::FORWARD );
	fsm->Update( 42 );
	fsm->Update( 930 );
	fsm->Update( 930 );
	fsm->Update( 930 );
	fsm->Update( 930 );
	fsm->ChangeState( FSM_STATE::IDLE );
	fsm->Update( 930 );
	fsm->ChangeState( FSM_STATE::FORWARD );
	fsm->Update( 930 );
	fsm->ChangeState( FSM_STATE::IDLE );
	fsm->Update( 930 );
	fsm->Update( 930 );
	fsm->Update( 930 );
	fsm->Update( 930 );
}

void Goblin::Idle_Before( float dt ) {
	fprintf( stdout, "Idle_Before\n" );
}

void Goblin::Idle_Update( float dt ) {
	fprintf( stdout, "Idle_Update\n" );
}

void Goblin::Idle_After( float dt ) {
	fprintf( stdout, "Idle_After\n" );
}

void Goblin::Forward_Before( float dt ) {
	fprintf( stdout, "Forward_Before\n" );
}

void Goblin::Forward_Update( float dt ) {
	fprintf( stdout, "Forward_Update\n" );
}

void Goblin::Forward_After( float dt ) {
	fprintf( stdout, "Forward_After\n" );
}

struct Foo {
	Foo( int num ) : num_( num ) {}
	void print_add( int i ) const { fprintf(stdout, "%i %i \n",num_,i); }
	int num_;
};

// store a call to a member function
//std::function<void( const Foo&, int )> f_add_display = &Foo::print_add;
const Foo foo( 314159 );
//f_add_display( foo, 1 );