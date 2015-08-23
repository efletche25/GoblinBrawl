#pragma once
#include "D3D11.h"
#include <DirectXMath.h>

using namespace DirectX;

namespace Vertex {
	struct SimpleVertex {
		XMFLOAT3 Pos;
	};
}

class InputLayoutDesc {
public:
	static const D3D11_INPUT_ELEMENT_DESC SimpleVertexDesc[1];
};

class InputLayouts {
public:
	static void InitAll( ID3D11Device* device );
	static void DestroyAll();
	static ID3D11InputLayout* Simple;
};
