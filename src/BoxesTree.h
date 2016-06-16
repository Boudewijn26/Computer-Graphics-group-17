//
// Created by boudewijn on 6/16/16.
//
#pragma once


#include "BoundingBox.h"

class BoundingBox;

class BoxesTree {

public:
	BoxesTree(BoundingBox& element);
	~BoxesTree();
	virtual bool findBox(Vec3Df origin, Vec3Df dest, BoundingBox& out) = 0;

protected:
	BoundingBox& element;

};


class BoxesNode : public BoxesTree {
public:
	BoxesNode(BoundingBox* element, BoxesTree* left, BoxesTree* right);
	bool findBox(Vec3Df origin, Vec3Df dest, BoundingBox& out);
	~BoxesNode();

private:
	BoxesTree* left;
	BoxesTree* right;
};

class BoxesEndpoint : public BoxesTree {
public:
	BoxesEndpoint(BoundingBox* element);
	bool findBox(Vec3Df origin, Vec3Df dest, BoundingBox& out);

};
