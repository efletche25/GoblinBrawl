#pragma once
#include <string>
#include <DirectXMath.h>
#include "d3dx11effect.h"
#include "Lighting.h"

struct PointLight;

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

class StaticGeomEffect : public Effect {
public:
	StaticGeomEffect( ID3D11Device* device, const std::wstring& filename );
	~StaticGeomEffect();
	void XM_CALLCONV SetWorld( DirectX::FXMMATRIX m ) { world->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldInvTranspose( DirectX::FXMMATRIX m ) { worldInvTranspose->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void XM_CALLCONV SetWorldViewProj( DirectX::FXMMATRIX m ) { worldViewProj->SetMatrix( reinterpret_cast<const float*>(&m) ); }
	void SetDiffuseMap( ID3D11ShaderResourceView* srv ) { diffuseMap->SetResource( srv ); }
	void XM_CALLCONV SetEyePosW( const DirectX::FXMVECTOR v ) { eyePosW->SetRawValue( &v, 0, sizeof( DirectX::XMFLOAT3 ) ); }
	void SetPointLights( const PointLight* lights ) { pointLights->SetRawValue( lights, 0, POINTLIGHT_COUNT *sizeof( PointLight ) ); }
	void SetMaterial( const Material& m ) { mat->SetRawValue( &m, 0, sizeof( Material ) ); }
	ID3DX11EffectTechnique*					staticGeomLight5Tech;
	ID3DX11EffectMatrixVariable*			world;
	ID3DX11EffectMatrixVariable*			worldInvTranspose;
	ID3DX11EffectMatrixVariable*			worldViewProj;
	ID3DX11EffectShaderResourceVariable*	diffuseMap;
	ID3DX11EffectVectorVariable*			eyePosW;
	ID3DX11EffectVariable*					pointLights;
	ID3DX11EffectVariable*					mat;
};

class Effects {
public:
	static void InitAll( ID3D11Device* device );
	static void DestroyAll();

	static SimpleEffect* SimpleFX;
	static TerrainEffect* TerrainFX;
	static StaticGeomEffect* StaticGeomFX;
};

