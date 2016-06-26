//
// Created by boudewijn on 6/16/16.
//

#include "BoundingTree.h"

bool findBox(Ray ray, BoundingBox*& out) {
	if (!element->doesIntersect(ray)) {
		return false;
	}

	BoundingBox* leftOut;
	BoundingBox* rightOut;
	bool leftIntersect = left->findBox(ray, leftOut);
	bool rightIntersect = right->findBox(ray, rightOut);

	if (leftIntersect == rightIntersect) {
		out = element;
		return true;
	} else if (leftIntersect) {
		out = leftOut;
		return true;
	} else if (rightIntersect) {
		out = rightOut;
		return true;
	}
	return false;
}