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
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	ID3DX11EffectTechnique*			simpleTechnique;
	ID3DX11EffectMatrixVariable*	worldViewProj;
};

class TerrainEffect : public Effect {
public:
	TerrainEffect( ID3D11Device* device, const std::wstring& filename );
	~TerrainEffect();
	void XM_CALLCONV SetWorld( DirectX::FXMMATRIX m ) { world->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldInvTranspose( DirectX::FXMMATRIX m ) { worldInvTranspose->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void SetDiffuseMap( ID3D11ShaderResourceView* srv ) { diffuseMap->SetResource( srv ); }
	ID3DX11EffectTechnique*					terrainTechnique;
	ID3DX11EffectMatrixVariable*			world;
	ID3DX11EffectMatrixVariable*			worldInvTranspose;
	ID3DX11EffectMatrixVariable*			worldViewProj;
	ID3DX11EffectShaderResourceVariable*	diffuseMap;
};

class Effects {
public:
	static void InitAll( ID3D11Device* device );
	static void DestroyAll();

	static SimpleEffect* SimpleFX;
	static TerrainEffect* TerrainFX;
};

