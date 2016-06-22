//
// Created by boudewijn on 6/16/16.
//

#include "BoxesTree.h"

bool BoxesNode::findBox(Vec3Df origin, Vec3Df dest, BoundingBox*& out) {
	if (!element->doesIntersect(origin, dest)) {
		return false;
	}

	BoundingBox* leftOut;
	BoundingBox* rightOut;
	bool leftIntersect = left->findBox(origin, dest, leftOut);
	bool rightIntersect = right->findBox(origin, dest, rightOut);

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

BoxesNode::BoxesNode(BoundingBox* element, BoxesTree* left, BoxesTree* right) : BoxesTree(element) {
	this->left = left;
	this->right = right;
}

BoxesNode::~BoxesNode() {
	delete left;
	delete right;
}


bool BoxesEndpoint::findBox(Vec3Df origin, Vec3Df dest, BoundingBox*& out) {
	if (element->doesIntersect(origin, dest)) {
		out = element;
		return true;
	}
	return false;
}

BoxesEndpoint::BoxesEndpoint(BoundingBox* element) : BoxesTree(element) {
}


BoxesTree::BoxesTree(BoundingBox* element) {
	this->element = element;
}

BoxesTree::~BoxesTree() {
	delete element;
}

