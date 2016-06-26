#pragma once

#include <vector>
#include "mesh.h"
#include <utility>
#include "BoundingTree.h"
#include "raytracing.h"

class BoundingTree;
struct Ray;

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
	BoundingTree* splitToTree(int threshold);
	bool doesIntersect(Ray ray);

	BoundingBox &operator=(const BoundingBox &other);

private:
	void init(std::vector<Vertex> vertices, std::vector<const Triangle*> triangles);

	const std::vector<Vertex>& vertices;
	std::vector<const Triangle*> triangles;
	Vec3Df bmin;
	Vec3Df bmax;

};