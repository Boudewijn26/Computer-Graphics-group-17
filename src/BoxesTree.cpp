//
// Created by boudewijn on 6/16/16.
//

#include "BoxesTree.h"

bool BoxesNode::findBox(Ray ray, BoundingBox*& out) {
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

BoxesNode::BoxesNode(BoundingBox* element, BoxesTree* left, BoxesTree* right) : BoxesTree(element) {
	this->left = left;
	this->right = right;
}

BoxesNode::~BoxesNode() {
	delete left;
	delete right;
}


bool BoxesEndpoint::findBox(Ray ray, BoundingBox*& out) {
	if (element->doesIntersect(ray)) {
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

