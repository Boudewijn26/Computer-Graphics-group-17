//
// Created by boudewijn on 6/16/16.
//

#include "BoxesTree.h"

bool BoxesNode::findBox(Vec3Df origin, Vec3Df dest, BoundingBox*& out) {
	if (element->doesIntersect(origin, dest)) {
		out = element;
		return true;
	}

	// This only works because the || shortcircuits
	// Otherwise the out could be from the second findBox call
	return left->findBox(origin, dest, out) || right->findBox(origin, dest, out);
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

