#include "BoundingStructure.h"
#include <limits>
#include <algorithm>
#include <stack>
#include "raytracing.h"

bool doesIntersect(BoundingBox * boundingBox, Ray ray) {
	Vec3Df t1 = ((*boundingBox).bmin - ray.origin) * ray.invdir;
	Vec3Df t2 = ((*boundingBox).bmax - ray.origin) * ray.invdir;

	float tmin = std::min(t1[0], t2[0]);
	float tmax = std::max(t1[0], t2[0]);

	tmin = std::max(tmin, std::min(t1[1], t2[1]));
	tmax = std::min(tmax, std::max(t1[1], t2[1]));

	tmin = std::max(tmin, std::min(t1[2], t2[2]));
	tmax = std::min(tmax, std::max(t1[2], t2[2]));

	return tmax > std::max(tmin, 0.0f);
}

BoundingBox * split(BoundingBox * boundingBox, int threshold) {
	switch ((*boundingBox).type) {
		case NONE: {
			std::cout << "ERROR; SPLIT CALLED ON BOUNDING BOX WITH TYPE NONE" << std::endl;
			return boundingBox;
			break; }
		case TRIANGLE: {
			std::vector<BoundingBox *> ttriangles = triangleSplit(boundingBox, threshold);
			BoundingBox * tr = &BoundingBox(ttriangles);
			if (ttriangles.size() > threshold) return split(tr, threshold);
			return tr;
			break; }
		case BOX: {
			std::vector<BoundingBox *> bboxes = boxSplit(boundingBox, threshold);
			BoundingBox * br = &BoundingBox(bboxes);
			if (bboxes.size() > threshold) return split(br, threshold);
			return br;
			break; }
		default: {
			std::cout << "CRITICAL ERROR; BOUNDING BOX TYPE " << (*boundingBox).type << " IS UNKNOWN" << std::endl;
			exit(EXIT_FAILURE);
			break; }
	}
}

std::vector<BoundingBox *> triangleSplit(BoundingBox * boundingBox, int threshold) {
	std::vector<BoundingBox *> r;

	if ((*boundingBox).type != TRIANGLE) {
		std::cout << "ERROR; TRIANGLESPLIT CALLED ON BOUNDING BOX WITH TYPE " << (*boundingBox).type << std::endl;
		r.push_back(boundingBox);
		return r;
	}

	std::vector<Triangle *> triangles = (*boundingBox).triangles;

	if (triangles.size() <= threshold) {
		r.push_back(boundingBox);
		return r;
	}

	Vec3Df size = (*boundingBox).bmax - (*boundingBox).bmin;
	unsigned int axis = (
		size[0] > size[1]
			? size[0] > size[2]
				? 0
				: 2
			: size[1] > size[2]
				? 1
				: 2
	);

	std::vector<Triangle *> firstSet;
	std::vector<Triangle *> secondSet;

	float splitPoint = ((*boundingBox).bmax[axis] + (*boundingBox).bmin[axis]) / 2.0f;

	for (Triangle * triangle : triangles) {
		bool inFirst = false;
		bool inSecond = false;
		for (int i = 0; i < 3; i++) {
			Vec3Df vertex = MyMesh.vertices[(*triangle).v[i]].p;
			if (!inFirst && vertex[axis] <= splitPoint) {
				firstSet.push_back(triangle);
				inFirst = true;
			}
			if (!inSecond && vertex[axis] >= splitPoint) {
				secondSet.push_back(triangle);
				inSecond = true;
			}
		}
	}

	BoundingBox firstBox = BoundingBox(firstSet);
	BoundingBox secondBox = BoundingBox(secondSet);
	
	std::vector<BoundingBox *> firstSplit = triangleSplit((&firstBox), threshold);
	std::vector<BoundingBox *> secondSplit = triangleSplit((&secondBox), threshold);

	firstSplit.insert(firstSplit.begin(), secondSplit.begin(), secondSplit.end());

	return firstSplit;
}

std::vector<BoundingBox *> boxSplit(BoundingBox * boundingBox, int threshold) {
	std::vector<BoundingBox *> r;

	if ((*boundingBox).type != BOX) {
		std::cout << "ERROR; TRIANGLESPLIT CALLED ON BOUNDING BOX WITH TYPE " << (*boundingBox).type << std::endl;
		r.push_back(boundingBox);
		return r;
	}

	std::vector<BoundingBox *> boxes = (*boundingBox).boxes;

	if (boxes.size() <= threshold) {
		r.push_back(boundingBox);
		return r;
	}

	Vec3Df size = (*boundingBox).bmax - (*boundingBox).bmin;
	unsigned int axis = (
		size[0] > size[1]
			? size[0] > size[2]
				? 0
				: 2
			: size[1] > size[2]
				? 1
				: 2
	);

	std::vector<BoundingBox *> firstSet;
	std::vector<BoundingBox *> secondSet;

	float splitPoint = ((*boundingBox).bmax[axis] + (*boundingBox).bmin[axis]) / 2.0f;

	for (BoundingBox * box : boxes) {
		bool inFirst = false;
		bool inSecond = false;

		if ((*box).bmin[axis] <= splitPoint) {
			firstSet.push_back(box);
			inFirst = true;
		}
		if (!inFirst && (*box).bmax[axis] <= splitPoint) {
			firstSet.push_back(box);
			inFirst = true;
		}
		if ((*box).bmin[axis] >= splitPoint) {
			secondSet.push_back(box);
			inSecond = true;
		}
		if (!inSecond && (*box).bmax[axis] >= splitPoint) {
			secondSet.push_back(box);
			inSecond = true;
		}
	}

	BoundingBox firstBox = BoundingBox(firstSet);
	BoundingBox secondBox = BoundingBox(secondSet);

	std::vector<BoundingBox *> firstSplit = boxSplit((&firstBox), threshold);
	std::vector<BoundingBox *> secondSplit = boxSplit((&secondBox), threshold);

	firstSplit.insert(firstSplit.begin(), secondSplit.begin(), secondSplit.end());

	return firstSplit;


}

std::vector<Triangle *> intersectingTriangles(BoundingBox * boundingBox, Ray ray) {
	std::stack<BoundingBox *> boundingBoxes;
	std::vector<Triangle *> r;
	boundingBoxes.push(boundingBox);
	while (boundingBoxes.size() > 0) {
		BoundingBox * box = boundingBoxes.top();
		boundingBoxes.pop();
		switch ((*box).type) {
		case NONE: {
			std::cout << "CRITICAL ERROR; INTERSECTION CALLED ON BOUNDING BOX WITH TYPE NONE" << std::endl;
			exit(EXIT_FAILURE);
			break; }
		case TRIANGLE: {
			for (Triangle * triangle : (*box).triangles) {
				r.push_back(triangle);
			}
			break; }
		case BOX: {
			for (BoundingBox * bbox : (*box).boxes) {
				if (doesIntersect(bbox, ray)) boundingBoxes.push(bbox);
			}
			break; }
		default: {
			std::cout << "CRITICAL ERROR; BOUNDING BOX TYPE " << (*boundingBox).type << " IS UNKNOWN" << std::endl;
			exit(EXIT_FAILURE);
			break; }
		}
	}
	return r;
}