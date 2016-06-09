#pragma once

#include <vector>
#include "mesh.h"
#include <utility>

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(const Mesh& mesh);
	BoundingBox(std::vector<Vertex> vertices, std::vector<Triangle> triangles);

	std::vector<Vec3Df> getVertices() const;
	std::vector<int> getDrawingIndices() const;

	std::vector<BoundingBox> split(int threshold);

private:
	void init(std::vector<Vertex> vertices, std::vector<Triangle> triangles);
	std::pair<BoundingBox, BoundingBox> doSplit();
	void split(std::vector<BoundingBox> &boxes, int threshold);

	std::vector<Vertex> vertices;
	std::vector<Triangle> triangles;
	Vec3Df origin;

	Vec3Df dimensions;
};