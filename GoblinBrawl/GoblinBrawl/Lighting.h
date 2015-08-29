#pragma once
#include <vector>
#include <DirectXMath.h>

#define POINTLIGHT_COUNT 5

class ModelLoader;

using namespace DirectX;

struct PointLight {
	XMFLOAT4 Ambient;
	XMFLOAT4 Diffuse;
	XMFLOAT4 Specular;

	//packed into 4D vector (Position, Range)
	XMFLOAT3 Position;
	float Range;

	// Packed into 4D vector (A0, A1, A2, Pad)
	XMFLOAT3 Att;
	float Pad;

};

struct Material {
	Material() { ZeroMemory( this, sizeof( this ) ); }
	XMFLOAT4 Ambient;
	XMFLOAT4 Diffuse;
	XMFLOAT4 Specular; // w = SpecPower
};


class Lighting {
public:
	Lighting();
	~Lighting();
	bool Init( ModelLoader* modelLoader );
	std::vector<PointLight> GetPointLights();
private:
	std::vector<PointLight>	pointLights;
};

