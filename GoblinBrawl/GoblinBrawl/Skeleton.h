#pragma once
#include "DirectXMath.h"
#include <string>
#include <map>

using namespace DirectX;

struct Bone {
	Bone::Bone() {
		localTransform = XMMatrixIdentity();
		finalTransform = XMMatrixIdentity();
	}
	std::string			name;
	int					idx;
	int					parentIdx;
	XMMATRIX			offset;
	XMMATRIX			localTransform;
	XMMATRIX			finalTransform;
};

class Skeleton {
public:
	Skeleton();
	~Skeleton();
	void AddBone( Bone* newBone );
	void XM_CALLCONV UpdateTransformByName( FXMVECTOR translate, FXMVECTOR rotQuat, FXMVECTOR scale, std::string name );
	void XM_CALLCONV UpdateTransformByIndex( FXMVECTOR translate, FXMVECTOR rotQuat, FXMVECTOR scale, int index );
	DirectX::XMFLOAT4X4* XM_CALLCONV GetFinalTransforms();
	Bone* GetBoneByName( std::string name );
	int BoneCount() { return numBones; };
private:
	int								numBones;
	std::map<int, Bone*>			idxBones;
	std::map<std::string, Bone*>	nameBones;
	DirectX::XMFLOAT4X4				finalTransformData[96];
};