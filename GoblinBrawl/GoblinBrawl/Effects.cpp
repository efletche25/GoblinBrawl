#include "stdafx.h"
#include "Effects.h"
#include <iostream>
#include <fstream>
#include <vector>
#include "d3dUtil.h"

Effect::Effect( ID3D11Device* device, const std::wstring& filename ) :
fx( nullptr ) {
	std::ifstream fin( filename, std::ios::binary );
	fin.seekg( 0, std::ios_base::end );
	int size = (int)fin.tellg();
	fin.seekg( 0, std::ios_base::beg );
	std::vector<char> compiledShader( size );
	fin.read( &compiledShader[0], size );
	fin.close();
	HR( D3DX11CreateEffectFromMemory( &compiledShader[0], size, 0, device, &fx ) );
}


Effect::~Effect() {
	ReleaseCOM( fx );
}

SimpleEffect::SimpleEffect( ID3D11Device* device, const std::wstring& filename ) :
Effect( device, filename ) {
	simpleTechnique = fx->GetTechniqueByName( "SimpleTech" );
	worldViewProj = fx->GetVariableByName( "gWorldViewProj" )->AsMatrix();
}
SimpleEffect::~SimpleEffect() {}

TerrainEffect::TerrainEffect( ID3D11Device* device, const std::wstring& filename ) :
Effect( device, filename ) {
	terrainTechnique = fx->GetTechniqueByName( "TerrainTech" );
	world = fx->GetVariableByName( "gWorld" )->AsMatrix();
	worldInvTranspose = fx->GetVariableByName( "gWorldInvTranspose" )->AsMatrix();
	worldViewProj = fx->GetVariableByName( "gWorldViewProj" )->AsMatrix();
	diffuseMap = fx->GetVariableByName( "gDiffuseMap" )->AsShaderResource();
}
TerrainEffect::~TerrainEffect() {}

SimpleEffect* Effects::SimpleFX = 0;
TerrainEffect* Effects::TerrainFX = 0;

void Effects::InitAll( ID3D11Device* device ) {
	SimpleFX = new SimpleEffect( device, L"fx/simple.fxo" );
	TerrainFX = new TerrainEffect( device, L"fx/terrain.fxo" );
}

void Effects::DestroyAll() {
	SafeDelete( SimpleFX );
	SafeDelete( TerrainFX );
}