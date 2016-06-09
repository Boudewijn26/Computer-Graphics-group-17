#pragma once

#include <vector>
#include "mesh.h"
#include <utility>

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(const Mesh& mesh);
	BoundingBox(std::vector<Vertex> vertices, std::vector<Triangle> triangles);

	std::vector<Vec3Df> getVertices();
	std::vector<int> getDrawingIndices();

	std::pair<BoundingBox, BoundingBox> doSplit();
private:
	void init(std::vector<Vertex> vertices, std::vector<Triangle> triangles);

	std::vector<Vertex> vertices;
	std::vector<Triangle> triangles;
	Vec3Df origin;
	Vec3Df dimensions;
};