#pragma once
#include "DirectX_11_1_Includes.h"
#include "Lighting.h"
#include "Keyboard.h"

struct ID3DX11Effect;
struct ID3DX11EffectTechnique;
struct ID3DX11EffectMatrixVariable;
struct ID3D11ShaderResourceView;

class ModelLoader;
class Mesh;
class Skeleton;
struct ID3D11DeviceContext;
struct ID3D11Device;

class Goblin {
public:
	enum PLAYER {
		PLAYER_1,
		PLAYER_2
	};

	Goblin();
	~Goblin();
	bool Init( ModelLoader* modelLoader, ID3D11Device* device, Keyboard::KeyboardStateTracker* kb, PLAYER player );
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
	Actions							action;
};

