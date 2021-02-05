#pragma once
// DirectXEffects
#include <d3dx11effect.h>
// DXUT includes
#include <DXUT.h>
#include <DXUTcamera.h>
// DirectXTK includes
#include "Effects.h"
#include "VertexTypes.h"
#include "PrimitiveBatch.h"
#include "GeometricPrimitive.h"
#include "util/vectorbase.h"
using namespace DirectX;
using namespace GamePhysics;
struct TriangleI {
	uint16_t i1, i2, i3;
};
class Mesh {
public:
	void calculate_triangles();
	void sample_mesh();
	std::vector<VertexPositionNormalTexture> vertices;
	std::vector<uint16_t> indices;
	std::vector<TriangleI> triangles;
	std::unique_ptr<GeometricPrimitive> prim = nullptr;
	std::vector<Vec3> samples;
};

