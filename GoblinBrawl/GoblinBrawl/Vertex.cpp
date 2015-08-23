#include "stdafx.h"
#include "Vertex.h"
#include "d3dx11effect.h"
#include "d3dUtil.h"
#include "Effects.h"

const D3D11_INPUT_ELEMENT_DESC InputLayoutDesc::SimpleVertexDesc[] =
{
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
};
const D3D11_INPUT_ELEMENT_DESC InputLayoutDesc::TerrainVertexDesc[] =
{
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 }
};

ID3D11InputLayout* InputLayouts::Simple = 0;
ID3D11InputLayout* InputLayouts::Terrain = 0;

void InputLayouts::InitAll( ID3D11Device* device ) {
	D3DX11_PASS_DESC passDesc;

	//Simple
	Effects::SimpleFX->simpleTechnique->GetPassByIndex( 0 )->GetDesc( &passDesc );
	HR( device->CreateInputLayout( InputLayoutDesc::SimpleVertexDesc, 1, passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &Simple ) );

	//Terrain
	Effects::TerrainFX->terrainTechnique->GetPassByIndex( 0 )->GetDesc( &passDesc );
	HR( device->CreateInputLayout( InputLayoutDesc::TerrainVertexDesc, 3, passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &Terrain ) );
}

void InputLayouts::DestroyAll() {
	ReleaseCOM( Simple );
}