#pragma once

#include <vector>
#include "mesh.h"
#include <utility>

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(const Mesh& mesh);
	BoundingBox(const std::vector<Vertex>& vertices, std::vector<Triangle> triangles);

	std::vector<Vec3Df> getVertices() const;
	std::vector<unsigned int> getDrawingIndices() const;
	std::vector<Triangle> getBoundingTriangles();

	std::vector<BoundingBox> split(int threshold);

private:
	void init(std::vector<Vertex> vertices, std::vector<Triangle> triangles);
	std::pair<BoundingBox, BoundingBox> doSplit();
	void split(std::vector<BoundingBox> &boxes, int threshold);

	const std::vector<Vertex>& vertices;
	std::vector<Triangle> triangles;
	Vec3Df origin;

	Vec3Df dimensions;
};