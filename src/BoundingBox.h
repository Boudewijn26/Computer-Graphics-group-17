#pragma once

#include <vector>
#include "mesh.h"
#include <utility>
#include "BoxesTree.h"

class BoxesTree;

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(const BoundingBox& box);
	BoundingBox(BoundingBox* box);
	BoundingBox(const Mesh& mesh);
	BoundingBox(const std::vector<Vertex>& vertices, std::vector<const Triangle*> triangles);

	std::vector<Vec3Df> getVertices() const;
	std::vector<unsigned int> getDrawingIndices() const;
	std::vector<Triangle> getBoundingTriangles();
	std::vector<const Triangle*>& getTriangles();

	std::vector<BoundingBox> split(int threshold);
	BoxesTree* splitToTree(int threshold);
	bool doesIntersect(Vec3Df origin, Vec3Df dest);

private:
	void init(std::vector<Vertex> vertices, std::vector<const Triangle*> triangles);
	std::pair<BoundingBox, BoundingBox> doSplit();
	void split(std::vector<BoundingBox> &boxes, int threshold);

	const std::vector<Vertex>& vertices;
	std::vector<const Triangle*> triangles;
	Vec3Df origin;

	Vec3Df dimensions;

};