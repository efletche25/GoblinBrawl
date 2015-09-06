#pragma once
#include "D3DX11.h"
#include <string>
#include "DirectXMath.h"
#include "Vertex.h"

class Mesh;
class Skeleton;

class FBXImporter {
public:
	FBXImporter( ID3D11Device* device, std::string modelDir, std::string textureDir );
	~FBXImporter();
	bool Load( std::string filename, Vertex::VERTEX_TYPE type );
	Mesh* GetMesh();
	Skeleton* GetSkeleton();
private:
	//void CreateIndexBuffer( const aiFace* indices, UINT count );
	//void CreateVertexBuffer( aiMesh* mesh, Vertex::VERTEX_TYPE type );
	//void ModelLoader::CreateSkeleton( aiBone** bones, int numBones );
	void CreateBoneHierarchy();
	//void FindBoneChildren( aiNode* node, int parentIdx );
	ID3D11Device*			device;
	std::string				modelDir;
	std::string				textureDir;
	ID3D11Buffer*			ib;
	UINT					indexCount;
	ID3D11Buffer*			vb;
};

