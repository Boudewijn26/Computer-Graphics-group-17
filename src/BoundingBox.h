#pragma once

#include <vector>
#include "mesh.h"

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(const Mesh& mesh);
	BoundingBox(const Vec3Df& origin, const Vec3Df& dimensions);
	std::vector<Vec3Df> getVertices();
	std::vector<int> getDrawingIndices();

private:
	void init(const Vec3Df& origin, const Vec3Df& dimensions);

	Vec3Df origin;
	Vec3Df dimensions;
};