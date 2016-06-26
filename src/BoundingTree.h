//
// Created by boudewijn on 6/16/16.
//
#pragma once


#include "BoundingBox.h"
#include "raytracing.h"

class BoundingBox;
struct Ray;

class BoundingTree {

public:
	BoundingTree(BoundingBox* element);
	bool findBox(Ray ray, BoundingBox*& out);

protected:
	BoundingBox* element;

};