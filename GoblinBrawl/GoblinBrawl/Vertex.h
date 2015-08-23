#pragma once
#include "D3D11.h"
#include <DirectXMath.h>

using namespace DirectX;

namespace Vertex {
	enum VERTEX_TYPE {
		SIMPLE,
		TERRAIN
	};
	struct SimpleVertex {
		XMFLOAT3 Pos;
	};
	struct TerrainVertex {
		XMFLOAT3 Pos;
		XMFLOAT3 Normal;
		XMFLOAT2 Tex;
	};
}

class InputLayoutDesc {
public:
	static const D3D11_INPUT_ELEMENT_DESC SimpleVertexDesc[1];
	static const D3D11_INPUT_ELEMENT_DESC TerrainVertexDesc[3];
};

class InputLayouts {
public:
	static void InitAll( ID3D11Device* device );
	static void DestroyAll();
	static ID3D11InputLayout* Simple;
	static ID3D11InputLayout* Terrain;
};
