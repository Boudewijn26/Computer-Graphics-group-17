#pragma once

#include <vector>
#include "Vec3D.h"

class BoundingBox {
public:
	BoundingBox(Vec3Df origin, Vec3Df dimensions);
	std::vector<Vec3Df> getVertices();
	std::vector<int> getDrawingIndices();

private:


	Vec3Df origin;
	Vec3Df dimensions;
};