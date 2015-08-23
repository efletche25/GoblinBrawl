#pragma once
#include <string>
#include "D3DX11.h"
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

class Mesh;

class ModelLoader {
public:
	ModelLoader( ID3D11Device* device, std::string modelDir, std::string textureDir );
	~ModelLoader();
	bool Load( std::string filename );
	Mesh* GetMesh();
private:
	void CreateIndexBuffer(const aiFace* indices, UINT count);
	void CreateVertexBuffer( const aiVector3D* vertices, UINT count );
	ID3D11Device*		device;
	std::string			modelDir;
	std::string			textureDir;
	const aiScene*		scene;
	ID3D11Buffer*		ib;
	UINT				indexCount;
	ID3D11Buffer*		vb;
};