#include "BoundingBox.h"
#include "BoxesTree.h"
#include <limits>
#include <algorithm>
#include "raytracing.h"
#include "raytracing.h"

BoundingBox::BoundingBox() : vertices(std::vector<Vertex>()) {
	bmin = Vec3Df(0, 0, 0);
	bmax = Vec3Df(0, 0, 0);
}

BoundingBox::BoundingBox(const BoundingBox& box) : vertices(box.vertices) {
	this->triangles = box.triangles;
	this->bmin = box.bmin;
	this->bmax = box.bmax;
}

BoundingBox::BoundingBox(const Mesh& mesh) : vertices(mesh.vertices) {
	for (std::vector<Triangle>::const_iterator it = mesh.triangles.begin(); it != mesh.triangles.end(); ++it) {
		const Triangle* trianglePtr = &*it;
		this->triangles.push_back(trianglePtr);
	}
	init(mesh.vertices, triangles);
}

BoundingBox::BoundingBox(const std::vector<Vertex>& vertices, std::vector<const Triangle*> triangles) : vertices(vertices) {
	this->triangles = triangles;
	init(vertices, triangles);
}

void BoundingBox::init(std::vector<Vertex> vertices, std::vector<const Triangle*> triangles) {
	Vec3Df min = Vec3Df(FLT_MAX, FLT_MAX, FLT_MAX);
	Vec3Df max = Vec3Df(FLT_MIN, FLT_MIN, FLT_MIN);

	for (std::vector<const Triangle*>::iterator it = triangles.begin(); it != triangles.end(); ++it) {
		Triangle triangle = **it;
		for (int i = 0; i < 3; i++) {
			Vec3Df vertex = vertices[triangle.v[i]].p;
			for (int j = 0; j < 3; j++) {
				min[j] = std::min(min[j], vertex[0]);
				max[j] = std::max(max[j], vertex[0]);
			}
		}
	}

	this->bmin = min;
	this->bmax = max;
}

std::pair<BoundingBox, BoundingBox> BoundingBox::doSplit() {
	int axis;
	Vec3Df dimensions = bmax - bmin;
	if (dimensions[0] > std::max(dimensions[1], dimensions[2])) {
		axis = 0;
	} else if (dimensions[1] > std::max(dimensions[0], dimensions[2])) {
		axis = 1;
	} else {
		axis = 2;
	}

	float sum = 0;
	for (std::vector<const Triangle*>::iterator it = triangles.begin(); it != triangles.end(); ++it) {
		const Triangle* triangle = *it;

		sum += vertices[triangle->v[0]].p[axis] + vertices[triangle->v[1]].p[axis] + vertices[triangle->v[2]].p[axis];

	}
	float splitPoint = sum / (triangles.size() * 3);

	std::vector<const Triangle*> firstBox;
	std::vector<const Triangle*> secondBox;

	for (std::vector<const Triangle*>::iterator it = triangles.begin(); it != triangles.end(); ++it) {
		const Triangle* triangle = *it;
		bool inFirst = false;
		bool inSecond = false;

		for (int i = 0; i < 3; i++) {
			float pointOnAxis = vertices[triangle->v[i]].p[axis];
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
	vertices.push_back(Vec3Df(bmin[0], bmin[1], bmin[2]));
	vertices.push_back(Vec3Df(bmin[0], bmin[1], bmax[2]));
	vertices.push_back(Vec3Df(bmin[0], bmax[1], bmin[2]));
	vertices.push_back(Vec3Df(bmin[0], bmax[1], bmax[2]));
	vertices.push_back(Vec3Df(bmax[0], bmin[1], bmin[2]));
	vertices.push_back(Vec3Df(bmax[0], bmin[1], bmax[2]));
	vertices.push_back(Vec3Df(bmax[0], bmax[1], bmin[2]));
	vertices.push_back(Vec3Df(bmax[0], bmax[1], bmax[2]));
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
		return;
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

BoxesTree* BoundingBox::splitToTree(int threshold) {
	if (triangles.size() < threshold) {
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

bool BoundingBox::doesIntersect(Ray ray) {
	Vec3Df t1 = (bmin - ray.origin) * ray.invdir;
	Vec3Df t2 = (bmax - ray.origin) * ray.invdir;

	float tmin = std::min(t1[0], t2[0]);
	float tmax = std::max(t1[0], t2[0]);

	tmin = std::max(tmin, std::min(t1[1], t2[1]));
	tmax = std::min(tmax, std::max(t1[1], t2[1]));

	tmin = std::max(tmin, std::min(t1[2], t2[2]));
	tmax = std::min(tmax, std::max(t1[2], t2[2]));

	return tmax > std::max(tmin, 0.0f);
}

BoundingBox &BoundingBox::operator=(const BoundingBox &other) {
	BoundingBox box = BoundingBox(
			other.vertices,
			other.triangles
	);
	return box;
}

std::vector<const Triangle*> &BoundingBox::getTriangles() {
	return triangles;
}

