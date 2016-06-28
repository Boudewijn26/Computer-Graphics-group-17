//
// Created by boudewijn on 6/16/16.
//

#include "BoxesTree.h"

bool BoxesNode::findBox(const Ray& ray, std::vector<BoundingBox*>& out) {
	if (!element->doesIntersect(ray)) {
		return false;
	}

	BoundingBox* leftOut;
	BoundingBox* rightOut;
	bool leftIntersect = left->findBox(ray, out);
	bool rightIntersect = right->findBox(ray, out);

	return leftIntersect || rightIntersect;
}

BoxesNode::BoxesNode(BoundingBox* element, BoxesTree* left, BoxesTree* right) : BoxesTree(element) {
	this->left = left;
	this->right = right;
}

BoxesNode::~BoxesNode() {
	delete left;
	delete right;
}


bool BoxesEndpoint::findBox(const Ray& ray, std::vector<BoundingBox*>& out) {
	if (element->doesIntersect(ray)) {
		out.push_back(element);
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

