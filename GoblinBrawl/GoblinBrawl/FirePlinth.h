#pragma once
#include <DirectXMath.h>
struct ID3DX11Effect;
struct ID3DX11EffectTechnique;
struct ID3DX11EffectMatrixVariable;
struct ID3D11ShaderResourceView;

class ModelLoader;
class Mesh;
class ID3D11DeviceContext;
class ID3D11Device;

class FirePlinth {
public:
	FirePlinth();
	~FirePlinth();
	bool Init( ModelLoader* modelLoader, ID3D11Device* device );
	void XM_CALLCONV Draw( DirectX::FXMMATRIX viewProj, ID3D11DeviceContext* context );
private:
	Mesh*							mesh;
	ID3D11ShaderResourceView*		diffuseView;
};
