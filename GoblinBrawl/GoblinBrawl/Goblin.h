#pragma once
#include "DirectX_11_1_Includes.h"
#include "Lighting.h"
#include "Keyboard.h"
#include "GamePad.h"
#include "FSM.h"

struct ID3DX11Effect;
struct ID3DX11EffectTechnique;
struct ID3DX11EffectMatrixVariable;
struct ID3D11ShaderResourceView;

class ModelLoader;
class Mesh;
class Skeleton;
struct ID3D11DeviceContext;
struct ID3D11Device;

class PhysicsWorld;
class btKinematicCharacterController;
class btPairCachingGhostObject;

class Goblin {
public:
	enum PLAYER {
		PLAYER_1 = 0,
		PLAYER_2 = 1
	};

	Goblin();
	~Goblin();
	bool Init( ModelLoader* modelLoader, ID3D11Device* device, Keyboard::KeyboardStateTracker* kb, GamePad* gamepad, PLAYER player, PhysicsWorld* physicsWorld );
	void Update(float dt);
	void XM_CALLCONV Draw( FXMMATRIX viewProj, FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context );
	void XM_CALLCONV SetPos( FXMVECTOR pos );
	FXMVECTOR XM_CALLCONV getPos();
	void XM_CALLCONV SetRot( FXMVECTOR rot );
	void ResetActions();
	void DebugActionDisplay();
private:
	struct Actions {
		bool Forward;
		bool Back;
		bool Left;
		bool Right;
		bool Attack;
		bool Jump;
		bool Duck;
	};
	void UpdateActions();
	void UpdateController(float dtS);
	void UpdateModelTransforms();

	// Finite State Machine Functions
	void InitFSM();
	void Idle_Before( float dt );
	void Idle_Update( float dt );
	void Idle_After( float dt );
	void Forward_Before( float dt );
	void Forward_Update( float dt );
	void Forward_After( float dt );

	PLAYER							player;
	Mesh*							mesh;
	Skeleton*						skeleton;
	ID3D11ShaderResourceView*		diffuseView;
	Material						mat;
	XMMATRIX						pos;
	XMMATRIX						rot;
	XMMATRIX						scale;
	XMMATRIX						world;
	Keyboard::KeyboardStateTracker*	kb;
	GamePad*						gamePad;
	Actions							action;
	PhysicsWorld*					physicsWorld;
	btKinematicCharacterController*	controller;
	btPairCachingGhostObject*		ghostObject;
	FSM<Goblin>*					fsm;

	//Player movement
	float							forwardAmount;
	float							turnAmount;
	float							strafeAmount;
	float							forwardSpeed;	// m/s
	float							turnSpeed;		// rot/s
	float							strafeSpeed;	// m/s
};

