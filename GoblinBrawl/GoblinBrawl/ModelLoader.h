#pragma once
#include <string>
#include <vector>
#include "DirectX_11_1_Includes.h"
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include "Vertex.h"
#include "Lighting.h"

class Mesh;
class Skeleton;
struct Anim;

class ModelLoader {
public:
	ModelLoader( ID3D11Device* device, std::string modelDir, std::string textureDir );
	~ModelLoader();
	bool Load( std::string filename, Vertex::VERTEX_TYPE type );
	Mesh* GetMesh();
	void GetMeshExtents( float &outMinX, float &outMaxX, float &outMinY, float &outMaxY, float &outMinZ, float &outMaxZ );
	Skeleton* GetSkeleton();
	std::vector<Anim*> GetAnimations();
	std::vector<PointLight> GetPointLights();
private:
	void CreateIndexBuffer( const aiFace* indices, UINT count );
	void CreateVertexBuffer( aiMesh* mesh, Vertex::VERTEX_TYPE type );
	void ModelLoader::CreateSkeleton( aiBone** bones, int numBones );
	void CreateBoneHierarchy();
	void CreateAnimations();
	DirectX::XMMATRIX XM_CALLCONV ConvertMatrix( aiMatrix4x4 inMat );
	void FindBoneChildren( aiNode* node, int parentIdx );
	inline void UpdateExtents( float x, float y, float z );
	ID3D11Device*			device;
	std::string				modelDir;
	std::string				textureDir;
	const aiScene*			scene;
	ID3D11Buffer*			ib;
	UINT					indexCount;
	ID3D11Buffer*			vb;
	std::vector<PointLight> pointLights;
	Skeleton*				skeleton;
	float					minX;
	float					minY;
	float					minZ;
	float					maxX;
	float					maxY;
	float					maxZ;
	std::vector<Anim*>		anims;

	template <typename VertexType>
	void SetVertices( ID3D11Device* device, UINT count, const VertexType* vertices ) {
		D3D11_BUFFER_DESC vbd;
		vbd.Usage = D3D11_USAGE_IMMUTABLE;
		vbd.ByteWidth = sizeof( VertexType ) * count;
		vbd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		vbd.CPUAccessFlags = 0;
		vbd.MiscFlags = 0;
		vbd.StructureByteStride = 0;

		D3D11_SUBRESOURCE_DATA vinitData;
		vinitData.pSysMem = vertices;

		HR( device->CreateBuffer( &vbd, &vinitData, &vb ) );
	}

	struct BoneWeight {
		BoneWeight::BoneWeight( int boneIndex, float weight ) : boneIndex( boneIndex ), weight( weight ) {}
		int boneIndex;
		float weight;
	};
};