#include "BoundingBox.h"
#include "BoxesTree.h"
#include <limits>
#include <algorithm>
#include "raytracing.h"


BoundingBox::BoundingBox() : vertices(std::vector<Vertex>()), triangles(std::vector<Triangle>()) {
	origin = Vec3Df(0, 0, 0);
	dimensions = Vec3Df(0, 0, 0);
}

BoundingBox::BoundingBox(const BoundingBox& box) : vertices(box.vertices), triangles(box.triangles) {
	this->triangleIndices = box.triangleIndices;
	this->dimensions = box.dimensions;
	this->origin = box.origin;
}

BoundingBox::BoundingBox(const Mesh& mesh) : vertices(mesh.vertices), triangles(mesh.triangles) {
	for (int i = 0; i < mesh.triangles.size(); i++) {
		triangleIndices.push_back(i);
	}
	init(mesh.vertices, triangleIndices);
}

BoundingBox::BoundingBox(
		const std::vector<Vertex>& vertices,
		const std::vector<Triangle>& triangles,
		std::vector<int> triangleIndices) : vertices(vertices), triangles(triangles){
	this->triangleIndices = triangleIndices;
	init(vertices, triangleIndices);
}

void BoundingBox::init(std::vector<Vertex> vertices, std::vector<int> triangleIndices) {
	float lowestX = std::numeric_limits<float>::max();
	float lowestY = lowestX;
	float lowestZ = lowestX;

	float highestX = std::numeric_limits<float>::min();
	float highestY = highestX;
	float highestZ = highestX;

	for (std::vector<int>::iterator it = triangleIndices.begin(); it != triangleIndices.end(); ++it) {
		Triangle triangle = triangles[*it];
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
	for (std::vector<int>::iterator it = triangleIndices.begin(); it != triangleIndices.end(); ++it) {
		Triangle triangle = triangles[*it];

		sum += vertices[triangle.v[0]].p[axis] + vertices[triangle.v[1]].p[axis] + vertices[triangle.v[2]].p[axis];

	}
	float splitPoint = sum / (triangleIndices.size() * 3);

	std::vector<int> firstBox;
	std::vector<int> secondBox;

	for (std::vector<int>::iterator it = triangleIndices.begin(); it != triangleIndices.end(); ++it) {
		Triangle triangle = triangles[*it];
		bool inFirst = false;
		bool inSecond = false;

		for (int i = 0; i < 3; i++) {
			float pointOnAxis = vertices[triangle.v[i]].p[axis];
			if (!inFirst && (pointOnAxis < splitPoint)) {
				firstBox.push_back(*it);
				inFirst = true;
			}
			if (!inSecond && (pointOnAxis > splitPoint)) {
				secondBox.push_back(*it);
				inSecond = true;
			}
		}
	}

	BoundingBox first = BoundingBox(vertices, triangles, firstBox);
	BoundingBox second = BoundingBox(vertices, triangles, secondBox);
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
	if (triangleIndices.size() < threshold) {
		boxes.push_back(BoundingBox(vertices, triangles, triangleIndices));
		return;
	}
	std::pair<BoundingBox, BoundingBox> pair = doSplit();
	if (pair.first.triangleIndices.size() < threshold) {
		boxes.push_back(pair.first);
	} else {
		pair.first.split(boxes, threshold);
	}
	if (pair.second.triangleIndices.size() < threshold) {
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

BoxesTree* BoundingBox::splitToTree(int threshold) {
	if (triangleIndices.size() < threshold) {
		BoxesEndpoint* result = new BoxesEndpoint(this);
		return result;
	} else {
		std::pair<BoundingBox, BoundingBox> pair = doSplit();
		BoundingBox* leftBox = new BoundingBox(pair.first);
		BoundingBox* rightBox = new BoundingBox(pair.second);
		BoxesTree* left = leftBox->splitToTree(threshold);
		BoxesTree* right = rightBox->splitToTree(threshold);
		BoxesNode* node = new BoxesNode(this, left, right);
		return node;
	}
}

bool BoundingBox::doesIntersect(Vec3Df rayOrigin, Vec3Df dest) {
	// After http://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
	Vec3Df min = origin;
	Vec3Df max = origin + dimensions;
	Vec3Df direction = dest - rayOrigin;
	float tmin = (min[0] - rayOrigin[0]) / direction[0];
	float tmax = (max[0] - rayOrigin[0]) / direction[0];

	if (tmin > tmax) std::swap(tmin, tmax);

	float tymin = (min[1] - rayOrigin[1]) / direction[1];
	float tymax = (max[1] - rayOrigin[1]) / direction[1];

	if (tymin > tymax) std::swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	float tzmin = (min[2] - rayOrigin[2]) / direction[2];
	float tzmax = (max[2] - rayOrigin[2]) / direction[2];

	if (tzmin > tzmax) std::swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax)
		tmax = tzmax;

	return true;
}

BoundingBox &BoundingBox::operator=(const BoundingBox &other) {
	BoundingBox box = BoundingBox(
			other.vertices,
			other.triangles,
			other.triangleIndices
	);
	return box;
}

std::vector<int> &BoundingBox::getTriangles() {
	return triangleIndices;
}

