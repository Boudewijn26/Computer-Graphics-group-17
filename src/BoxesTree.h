//
// Created by boudewijn on 6/16/16.
//
#pragma once


#include "BoundingBox.h"

class Ray
{
public:
	Ray(const Vec3Df &orig, const Vec3Df &dir) : orig(orig), dir(dir)
	{
		invdir = Vec3Df(1 / dir[0], 1 / dir[1], 1 / dir[2]);
		sign[0] = (invdir[0] < 0);
		sign[1] = (invdir[1] < 0);
		sign[2] = (invdir[2] < 0);
	}
	Vec3Df orig, dir;       // ray orig and dir 
	Vec3Df invdir;
	int sign[3];
};

class BoundingBox;

class BoxesTree {

public:
	BoxesTree(BoundingBox* element);
	~BoxesTree();
	virtual bool findBox(const Ray &ray, std::vector<BoundingBox*>& out) = 0;
	bool findBox(const Vec3Df& origin, const Vec3Df& dest, std::vector<BoundingBox*>& out) {
		return findBox(Ray(origin, dest), out);
	}

protected:
	BoundingBox* element;

};


class BoxesNode : public BoxesTree {
public:
	BoxesNode(BoundingBox* element, BoxesTree* left, BoxesTree* right);
	bool findBox(const Ray &ray, std::vector<BoundingBox*>& out);
	~BoxesNode();

private:
	BoxesTree* left;
	BoxesTree* right;
};

class BoxesEndpoint : public BoxesTree {
public:
	BoxesEndpoint(BoundingBox* element);
	bool findBox(const Ray &ray, std::vector<BoundingBox*>& out);

};
