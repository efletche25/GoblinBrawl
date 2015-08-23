#pragma once
#include <string>
#include <DirectXMath.h>
#include "d3dx11effect.h"

class Effect {
public:
	Effect( ID3D11Device* device, const std::wstring& filename );
	virtual ~Effect();

private:
	Effect( const Effect& rhs );
	Effect& operator=(const Effect& rhs);

protected:
	ID3DX11Effect* fx;
};

class SimpleEffect : public Effect {
public:
	SimpleEffect( ID3D11Device* device, const std::wstring& filename );
	~SimpleEffect();
	void XM_CALLCONV SetWorldWiewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	ID3DX11EffectTechnique*			simpleTechnique;
	ID3DX11EffectMatrixVariable*	worldViewProj;
	//ID3DX11EffectShaderResourceVariable* DiffuseMap;
};

class Effects {
public:
	static void InitAll( ID3D11Device* device );
	static void DestroyAll();

	static SimpleEffect* SimpleFX;
};

