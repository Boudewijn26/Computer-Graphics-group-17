#include "Vec3D.h"

// void drawFresnel(Vec3Df incident, Vec3Df normal, double refIndex1, double refIndex2) {
//	double reflectance = SchlickReflectance(incident, normal, refIndex1, refIndex2);
//	draw reflection(incident, normal) with reduced intensity (reflectance);
//	draw refraction(incident, normal) with reduced intensity (1.0-reflectance)
// }

Vec3Df reflection(Vec3Df incident, Vec3Df normal) {
	double cosIncidence = -Vec3Df::dotProduct(incident, normal);
	return incident + 2 * cosIncidence * normal;
}

Vec3Df refraction(Vec3Df incident, Vec3Df normal, double refIndex1, double refIndex2) {
	double n = refIndex1 / refIndex2;
	double cosIncidence = -Vec3Df::dotProduct(incident, normal);
	double sinRefractionSquared = n*n * (1.0 - cosIncidence*cosIncidence);

	// Total internal reflection
	if (sinRefractionSquared > 1.0) return reflection(incident, normal);

	double cosRefraction = sqrt(1.0 - sinRefractionSquared);

	return n * incident + (n * cosIncidence - cosRefraction) * normal;
}

double SchlickReflectance(Vec3Df incident, Vec3Df normal, double refIndex1, double refIndex2) {
	// r0 = ( (n1-n2)/(n1+n2) )^2
	double r0 = ((refIndex1 - refIndex2) / (refIndex1 + refIndex2)) * ((refIndex1 - refIndex2) / (refIndex1 + refIndex2));
	double cosIncidence = -Vec3Df::dotProduct(incident, normal); 
	double subCos;
	if (refIndex1 > refIndex2) {
		double n = refIndex1 / refIndex2;
		double sinRefractionSquared = n*n * (1.0 - cosIncidence*cosIncidence);

		// Total internal reflection
		if (sinRefractionSquared > 1.0) return 1.0;

		double cosRefraction = sqrt(1.0 - sinRefractionSquared);

		subCos = 1.0 - cosRefraction;
	} else {
		subCos = 1.0 - cosIncidence;
	}
	return r0 + (1.0 - r0) * subCos * subCos * subCos * subCos * subCos;
}