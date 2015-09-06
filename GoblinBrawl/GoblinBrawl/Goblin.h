#pragma once
#include <DirectXMath.h>
#include "Lighting.h"

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
	Goblin();
	~Goblin();
	bool Init( ModelLoader* modelLoader, ID3D11Device* device );
	void Update(float dt);
	void XM_CALLCONV Draw( FXMMATRIX viewProj, FXMVECTOR cameraPos, std::vector<PointLight> pointLights, ID3D11DeviceContext* context );
	void XM_CALLCONV SetPos( FXMVECTOR pos );
	FXMVECTOR XM_CALLCONV getPos();
	void XM_CALLCONV SetRot( FXMVECTOR rot );
private:
	Mesh*							mesh;
	Skeleton*						skeleton;
	ID3D11ShaderResourceView*		diffuseView;
	Material						mat;
	XMMATRIX						pos;
	XMMATRIX						rot;
	XMMATRIX						scale;
	XMMATRIX						world;
};

