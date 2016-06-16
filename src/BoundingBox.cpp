#include "BoundingBox.h"
#include <limits>
#include <algorithm>

BoundingBox::BoundingBox() : vertices(std::vector<Vertex>()) {
	origin = Vec3Df(0, 0, 0);
	dimensions = Vec3Df(0, 0, 0);
}

BoundingBox::BoundingBox(const Mesh& mesh) : vertices(mesh.vertices) {
	init(mesh.vertices, mesh.triangles);
}

BoundingBox::BoundingBox(const std::vector<Vertex>& vertices, std::vector<Triangle> triangles) : vertices(vertices) {
	init(vertices, triangles);
}

void BoundingBox::init(std::vector<Vertex> vertices, std::vector<Triangle> triangles) {
	float lowestX = std::numeric_limits<float>::max();
	float lowestY = lowestX;
	float lowestZ = lowestX;

	float highestX = std::numeric_limits<float>::min();
	float highestY = highestX;
	float highestZ = highestX;

	for (std::vector<Triangle>::iterator it = triangles.begin(); it != triangles.end(); ++it) {
		Triangle triangle = *it;
		for (int i = 0; i < 3; i++) {
			Vec3Df vertex = vertices[triangle.v[i]].p;
			lowestX = std::min(lowestX, vertex[0]);
			highestX = std::max(highestX, vertex[0]);

			lowestY = std::min(lowestY, vertex[1]);
			highestY = std::max(highestY, vertex[1]);

			lowestZ = std::min(lowestZ, vertex[2]);
			highestZ = std::max(highestZ, vertex[2]);
		}
	}

	this->origin = Vec3Df(lowestX, lowestY, lowestZ);
	Vec3Df farCorner = Vec3Df(highestX, highestY, highestZ);
	this->dimensions = farCorner - origin;

	this->triangles = triangles;
}

std::pair<BoundingBox, BoundingBox> BoundingBox::doSplit() {
	int axis;

	if (dimensions[0] > std::max(dimensions[1], dimensions[2])) {
		axis = 0;
	} else if (dimensions[1] > std::max(dimensions[0], dimensions[2])) {
		axis = 1;
	} else {
		axis = 2;
	}

	float sum = 0;
	for (std::vector<Triangle>::iterator it = triangles.begin(); it != triangles.end(); ++it) {
		Triangle triangle = *it;

		sum += vertices[triangle.v[0]].p[axis] + vertices[triangle.v[1]].p[axis] + vertices[triangle.v[2]].p[axis];

	}
	float splitPoint = sum / (triangles.size() * 3);

	std::vector<Triangle> firstBox;
	std::vector<Triangle> secondBox;

	for (std::vector<Triangle>::iterator it = triangles.begin(); it != triangles.end(); ++it) {
		Triangle triangle = *it;
		bool inFirst = false;
		bool inSecond = false;

		for (int i = 0; i < 3; i++) {
			float pointOnAxis = vertices[triangle.v[i]].p[axis];
			if (!inFirst && (pointOnAxis < splitPoint)) {
				firstBox.push_back(triangle);
				inFirst = true;
			}
			if (!inSecond && (pointOnAxis > splitPoint)) {
				secondBox.push_back(triangle);
				inSecond = true;
			}
		}
	}

	BoundingBox first = BoundingBox(vertices, firstBox);
	BoundingBox second = BoundingBox(vertices, secondBox);
	return std::pair<BoundingBox, BoundingBox>(first, second);
}

std::vector<Vec3Df> BoundingBox::getVertices() const {
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

std::vector<unsigned int> BoundingBox::getDrawingIndices() const {
	std::vector<unsigned int> indices;
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

void BoundingBox::split(std::vector<BoundingBox>& boxes, int threshold) {
	if (triangles.size() < threshold) {
		boxes.push_back(BoundingBox(vertices, triangles));
	}
	std::pair<BoundingBox, BoundingBox> pair = doSplit();
	if (pair.first.triangles.size() < threshold) {
		boxes.push_back(pair.first);
	} else {
		pair.first.split(boxes, threshold);
	}
	if (pair.second.triangles.size() < threshold) {
		boxes.push_back(pair.second);
	} else {
		pair.second.split(boxes, threshold);
	}
}

std::vector<BoundingBox> BoundingBox::split(int threshold) {
	std::vector<BoundingBox> boxes;

	split(boxes, threshold);
	return boxes;
}

std::vector<Triangle> BoundingBox::getBoundingTriangles() {
	std::vector<unsigned int> drawingIndices = getDrawingIndices();
	std::vector<Triangle> triangles;
	for (int i = 0; i < drawingIndices.size(); i += 3) {
		triangles.push_back(Triangle(
				drawingIndices[i],
				0,
				drawingIndices[i + 1],
				0,
				drawingIndices[i + 2],
				0
		));
	}

	return triangles;
}