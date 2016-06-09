#include "BoundingBox.h" 

BoundingBox::BoundingBox(Vec3Df origin, Vec3Df dimensions) {
	this->origin = origin;
	this->dimensions = dimensions;
}

std::vector<Vec3Df> BoundingBox::getVertices() {
	std::vector<Vec3Df> vertices;
	vertices.push_back(origin);
	vertices.push_back(Vec3Df(origin[0], origin[1], origin[2] + dimensions[2]));
	vertices.push_back(Vec3Df(origin[0], origin[1] + dimensions[1], origin[2]));
	vertices.push_back(Vec3Df(origin[0], origin[1] + dimensions[1], origin[2] + dimensions[2]));
	vertices.push_back(Vec3Df(origin[0] + dimensions[0], origin[1], origin[2]));
	vertices.push_back(Vec3Df(origin[0] + dimensions[0], origin[1], origin[2] + dimensions[2]));
	vertices.push_back(Vec3Df(origin[0] + dimensions[0], origin[1] + dimensions[1], origin[2]));
	vertices.push_back(origin + dimensions);
	return vertices;
}

std::vector<int> BoundingBox::getDrawingIndices() {
	std::vector<int> indices;
	indices.push_back(0);
	indices.push_back(1);
	indices.push_back(2);

	indices.push_back(2);
	indices.push_back(1);
	indices.push_back(3);

	indices.push_back(0);
	indices.push_back(2);
	indices.push_back(4);

	indices.push_back(6);
	indices.push_back(4);
	indices.push_back(2);

	indices.push_back(6);
	indices.push_back(2);
	indices.push_back(3);

	indices.push_back(7);
	indices.push_back(6);
	indices.push_back(3);

	indices.push_back(5);
	indices.push_back(4);
	indices.push_back(6);

	indices.push_back(7);
	indices.push_back(5);
	indices.push_back(6);

	indices.push_back(0);
	indices.push_back(4);
	indices.push_back(1);

	indices.push_back(4);
	indices.push_back(5);
	indices.push_back(1);

	indices.push_back(7);
	indices.push_back(3);
	indices.push_back(1);

	indices.push_back(1);
	indices.push_back(5);
	indices.push_back(7);
	return indices;
}