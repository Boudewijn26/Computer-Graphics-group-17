#pragma once
#include "raytracing.h"
#include <vector>
#include <algorithm>

struct Ray;

extern Mesh MyMesh;

enum BoundingType { TRIANGLE, BOX, NONE };

struct BoundingBox {
	std::vector<Triangle *> triangles;
	std::vector<BoundingBox *> boxes;
	BoundingType type = NONE;
	Vec3Df bmin;
	Vec3Df bmax;

	BoundingBox(Vec3Df min, Vec3Df max) {
		bmin = min;
		bmax = max;
	}

	BoundingBox(std::vector<Triangle *> boundingObjects) {
		type = TRIANGLE;
		triangles = boundingObjects;

		Vec3Df min = Vec3Df(FLT_MAX, FLT_MAX, FLT_MAX);
		Vec3Df max = Vec3Df(FLT_MIN, FLT_MIN, FLT_MIN);

		for (Triangle * triangle : triangles) {
			for (int i = 0; i < 3; i++) {
				Vec3Df vertex = MyMesh.vertices[(*triangle).v[i]].p;
				for (int j = 0; j < 3; j++) {
					min[j] = (std::min)(min[j], vertex[0]);
					max[j] = (std::max)(max[j], vertex[0]);
				}
			}
		}

		bmin = min;
		bmax = max;
	}

	BoundingBox(std::vector<Triangle> boundingObjects) {
		std::vector<Triangle *> newBoundingObjects;
		for (Triangle triangle : boundingObjects) {
			newBoundingObjects.push_back(&triangle);
		}
		
		type = TRIANGLE;
		triangles = newBoundingObjects;

		Vec3Df min = Vec3Df(FLT_MAX, FLT_MAX, FLT_MAX);
		Vec3Df max = Vec3Df(FLT_MIN, FLT_MIN, FLT_MIN);

		for (Triangle * triangle : triangles) {
			for (int i = 0; i < 3; i++) {
				Vec3Df vertex = MyMesh.vertices[(*triangle).v[i]].p;
				for (int j = 0; j < 3; j++) {
					min[j] = (std::min)(min[j], vertex[0]);
					max[j] = (std::max)(max[j], vertex[0]);
				}
			}
		}

		bmin = min;
		bmax = max;

	}

	BoundingBox(std::vector<BoundingBox *> boundingObjects) {
		type = BOX;
		boxes = boundingObjects;

		Vec3Df min = Vec3Df(FLT_MAX, FLT_MAX, FLT_MAX);
		Vec3Df max = Vec3Df(FLT_MIN, FLT_MIN, FLT_MIN);

		for (BoundingBox * box : boxes) {
			for (int i = 0; i < 3; i++) {
				min[i] = (std::min)(min[i], (*box).bmin[i]);
				max[i] = (std::max)(max[i], (*box).bmax[0]);
			}
		}

		bmin = min;
		bmax = max;
	}

};

bool doesIntersect(BoundingBox * boundingBox, Ray ray);

BoundingBox * split(BoundingBox * boundingBox, int threshold);
std::vector<BoundingBox *> triangleSplit(BoundingBox * boundingBox, int threshold);
std::vector<BoundingBox *> boxSplit(BoundingBox * boundingBox, int threshold);

std::vector<Triangle *> intersectingTriangles(BoundingBox * boundingBox, Ray ray);