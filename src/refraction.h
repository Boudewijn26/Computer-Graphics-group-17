#pragma once
#include "Vec3D.h"

// Return a (not normalized) reflection vector;
Vec3Df reflection(Vec3Df incident, Vec3Df normal);

// Return a (not normalized) refraction vector when possible, otherwise return the reflection;
Vec3Df refraction(Vec3Df incident, Vec3Df normal, double refIndex1, double refIndex2);

// Return the Schlick approximation of the reflectance according to the Fresnel equations.
double SchlickReflectance(Vec3Df incident, Vec3Df normal, double refIndex1, double refIndex2);