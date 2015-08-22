#pragma once
#include <DirectXMath.h>
using namespace DirectX;
class Camera {
public:
	Camera();
	~Camera();
	void Init(float aspectRatio);

	// Update expects the charactor pos and dir. The camera offset and
	// target will be calculated by the Update function.
	void XM_CALLCONV Update( FXMVECTOR pos, FXMVECTOR dir );
	XMMATRIX XM_CALLCONV GetViewProj();
private:
	XMMATRIX view;
	XMMATRIX proj;
	XMVECTOR up;
	float targetOffset;
	XMMATRIX viewProj;
	FLOAT nearZ;
	FLOAT farZ;
	FLOAT fovAngleY;
};

