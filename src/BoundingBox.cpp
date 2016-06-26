#include "BoundingBox.h"
#include "BoundingTree.h"
#include <limits>
#include <algorithm>
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
	std::cout << "init" << std::endl;
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

std::vector<Vec3Df> BoundingBox::getVertices() const {
	std::cout << "getVertices" << std::endl;
	std::vector<Vec3Df> vertices;
	vertices.push_back(Vec3Df(bmin[0], bmin[1], bmin[2])); // 
	vertices.push_back(Vec3Df(bmin[0], bmin[1], bmax[2])); // 
	vertices.push_back(Vec3Df(bmin[0], bmax[1], bmin[2])); // 
	vertices.push_back(Vec3Df(bmin[0], bmax[1], bmax[2])); // 
	vertices.push_back(Vec3Df(bmax[0], bmin[1], bmin[2])); // 
	vertices.push_back(Vec3Df(bmax[0], bmin[1], bmax[2])); // 
	vertices.push_back(Vec3Df(bmax[0], bmax[1], bmin[2])); // 
	vertices.push_back(Vec3Df(bmax[0], bmax[1], bmax[2])); // 
	return vertices;
}

std::vector<unsigned int> BoundingBox::getDrawingIndices() const {
	std::cout << "getDrawingIndices" << std::endl;
	std::vector<unsigned int> indices;

	unsigned int triangles[12][3] = {{0,1,2},{0,1,4},{0,2,4},{1,2,3},{2,4,6},{1,4,5},{1,3,5},{1,3,7},{2,3,6},{3,6,7},{4,5,6},{5,6,7}};

	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 3; j++) {
			indices.push_back(triangles[i][j]);
		}
	}
	
	return indices;
}

std::vector<BoundingBox> BoundingBox::split(int threshold) {
	std::cout << "split1" << std::endl;
	std::vector<BoundingBox> boxes;

	if (triangles.size() < threshold) {
		boxes.push_back(BoundingBox(vertices, triangles));
		return boxes;
	}

	for (int x = 0; x < 2; x++) {
		for (int y = 0; y < 2; y++) {
			for (int z = 0; z < 2; z++) {

			}
		}
	}
	std::cout << pair.first.triangles.size() << ":" << pair.second.triangles.size() << std::endl;
	if (pair.first.triangles.size() < threshold) {
		boxes.push_back(pair.first);
	}
	else {
		pair.first.split(boxes, threshold);
	}
	if (pair.second.triangles.size() < threshold) {
		boxes.push_back(pair.second);
	}
	else {
		pair.second.split(boxes, threshold);
	}

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

BoundingTree* BoundingBox::splitToTree(int threshold) {
	
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

