#pragma once

#include <vector>
#include "mesh.h"
#include <utility>
#include "BoxesTree.h"

class BoxesTree;
class Ray;

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(const BoundingBox& box);
	BoundingBox(BoundingBox* box);
	BoundingBox(const Mesh& mesh);
	BoundingBox(const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles, std::vector<int> triangleIndices);

	std::vector<Vec3Df> getVertices() const;
	std::vector<unsigned int> getDrawingIndices() const;
	std::vector<Triangle> getBoundingTriangles();
	std::vector<int>& getTriangles();

	std::vector<BoundingBox> split(int threshold);
	BoxesTree* splitToTree(int threshold);
	bool doesIntersect(const Ray& ray);

	const Vec3Df& getOrigin();
	const Vec3Df& getDimensions();


	BoundingBox &operator=(const BoundingBox &other);

private:
	void init(std::vector<Vertex> vertices, std::vector<int> triangleIndices);
	std::pair<BoundingBox, BoundingBox> doSplit();
	void split(std::vector<BoundingBox> &boxes, int threshold);

	const std::vector<Vertex>& vertices;
	const std::vector<Triangle>& triangles;
	std::vector<int> triangleIndices;
	Vec3Df origin;

	Vec3Df dimensions;

};