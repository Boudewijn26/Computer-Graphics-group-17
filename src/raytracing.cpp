#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include "BoxesTree.h"
#include <GL/glut.h>
#include <float.h>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include "raytracing.h"

using namespace std;


//temporary variables
//these are only used to illustrate
//a simple debug drawing. A ray
Vec3Df testRayOrigin;
Vec3Df testRayDestination;

Vec3Df sunVector;
rgb sunColor;

float pitchAngle = 0;
float yawAngle = 0;

BoundingBox box = BoundingBox();
BoxesTree* tree;

vector<Vec3Df> meshPoints;

// Specifies the distance of the sun from the origin
// This is usefull if the sun is drawn inside objects in the debug view
float sunDist = 4;

void drawBox(BoundingBox box);

bool isTriangleHit(const Vec3Df &origin, const Vec3Df &dest, const Vec3Df &v0, const Vec3Df &v1, const Vec3Df &v2);

Vec3Df calculateSunVector();

rgb sunVectorToRgb(Vec3Df sunVector);

void calculateSun();

//use this function for any preprocessing of the mesh.
void init()
{
	//load the mesh file
	//please realize that not all OBJ files will successfully load.
	//Nonetheless, if they come from Blender, they should, if they
	//are exported as WavefrontOBJ.
	//PLEASE ADAPT THE LINE BELOW TO THE FULL PATH OF THE dodgeColorTest.obj
	//model, e.g., "C:/temp/myData/GraphicsIsFun/dodgeColorTest.obj",
	//otherwise the application will not load properly
    MyMesh.loadMesh("models/rgbcubes.obj", true);
	MyMesh.computeVertexNormals();
    meshPoints = getVerticePoints(MyMesh.vertices);
	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera

    calculateSun();

    BoundingBox main = BoundingBox(MyMesh);
	BoundingBox* mainTree = new BoundingBox(main);
	tree = mainTree->splitToTree(500);
}

/**
 * True if the vector is the zero vector
 */
bool isZero(Vec3Df v) {
    return v[0] == 0 && v[1] == 0 && v[2] == 0;
}

void calculateSun() {
    sunVector = calculateSunVector();
    MyLightPositions.clear();
    MyLightPositions.push_back(sunVector);
    sunVector.normalize();
    sunColor = sunVectorToRgb(sunVector);
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	Vec3Df result;
	if (trace(origin, dest, 0, result)) return result;
	return Vec3Df(0, 0, 0);
}

bool trace(const Vec3Df & origin, const Vec3Df & dest, int level, Vec3Df& result) {
	Intersection intersection = Intersection();
	Vec3Df intersect;
	std::vector<BoundingBox*> hits;
	bool foundBox = tree->findBox(origin, dest, hits);
	if (!foundBox) {
		return false;
	}
	vector<int> triangles = vector<int>();

	for (std::vector<BoundingBox*>::iterator it = hits.begin(); it != hits.end(); ++it) {
		std::vector<int> &curTriangles = (*it)->getTriangles();
		triangles.reserve(triangles.size() + curTriangles.size());
		triangles.insert(triangles.end(), curTriangles.begin(), curTriangles.end());
	}

	std::vector<int>::iterator it;
	it = std::unique(triangles.begin(), triangles.end());
	triangles.resize((unsigned long) distance(triangles.begin(), it));

	for (int i=0; i < triangles.size(); ++i) {
        Triangle triangle = MyMesh.triangles[triangles[i]];
        if (intersectionPoint(origin, dest, meshPoints, triangle, intersect)) {
			float distance = (intersect - origin).getLength();
			if (intersection.distance > distance) {
				intersection.distance = distance;
				intersection.index = triangles[i];
				intersection.intersect = intersect;
				intersection.normal =
					Vec3Df::crossProduct(MyMesh.vertices[triangle.v[1]].p - MyMesh.vertices[triangle.v[0]].p,
										MyMesh.vertices[triangle.v[2]].p - MyMesh.vertices[triangle.v[0]].p);
				intersection.normal.normalize();
			}
        }
    }
	if (intersection.index == -1) return false;

	result = shade(intersection, level);

	return true;
}

Vec3Df diffuse(const Vec3Df & vertexPos, Vec3Df & normal, Material* material, Vec3Df lightPos) {
	Vec3Df diffuse = Vec3Df(0,0,0);
	normal.normalize();
	lightPos.normalize();

    Vec3Df lightVector = (lightPos - vertexPos); // Get the light position relative from the vertex position
    lightVector.normalize(); // And normalize it

	// Calculate the material colors with the sun color.
    Vec3Df materialWithSun = Vec3Df(
			(float) (sunColor.r * material->Kd().p[0]),
			(float) (sunColor.g * material->Kd().p[1]),
			(float) (sunColor.b * material->Kd().p[2])
	);

    // Calculate the diffusion (including color)
    diffuse += materialWithSun * fmax(Vec3Df::dotProduct(normal, lightPos), 0.0f);

	return diffuse;
}

Vec3Df blinnPhong(const Vec3Df & vertexPos, Vec3Df & normal, Material* material, Vec3Df lightPos) {
	Vec3Df specularity = Vec3Df(0,0,0);
	normal.normalize();
	lightPos.normalize();

    Vec3Df viewVector = (MyCameraPosition - vertexPos); // Get the view vector
    viewVector.normalize(); // And normalize it

    Vec3Df lightVector = (lightPos - vertexPos); // Get the light vector (opposite direction of viewvector.
    lightVector.normalize(); // And normalize it

    // Calculate the halfway vector
    Vec3Df halfwayVector = viewVector + lightVector;
    halfwayVector.normalize();

    float specTerm = max(Vec3Df::dotProduct(halfwayVector, normal), 0.0f);
    specTerm = pow(specTerm, material->Ns());

	Vec3Df materialWithSun = Vec3Df(
			(float) (sunColor.r * material->Ks().p[0]),
			(float) (sunColor.g * material->Ks().p[1]),
			(float) (sunColor.b * material->Ks().p[2])
	);

    specularity += materialWithSun * specTerm;

	return specularity;
}

Material getMat(int index) {
	int matIndex = MyMesh.triangleMaterials[index];
	return MyMesh.materials[matIndex];
}

Vec3Df shade(Intersection intersection, int level) {
    Vec3Df refl = Vec3Df(0,0,0);
	Vec3Df refr = Vec3Df(0,0,0);
	Vec3Df result = Vec3Df(0,0,0);

	Material material = getMat(intersection.index);

	/* Start of shading block */
    if (material.has_Ka()) {
		result += material.Ka();
    }

    if (material.has_Kd()) {
		result += diffuse(intersection.intersect, intersection.normal, &material, sunVector);
	}

	if (material.has_Ks()) {
		result += blinnPhong(intersection.intersect, intersection.normal, &material, sunVector);
	}
    /* End of shading block */

  /*  if(level<2) // && reflects
    {
        //calculate reflection vector
       refl = trace(hit, Vec3Df(0,0,0)reflection, level +1);
    }

    else if(level<2) // && refracts
    {
        //calculate refraction vector
       refr = trace(hit, Vec3Df(0,0,0)refraction, level+1);
    } */

	if (result.p[0] > 1) {
		result.p[0] = 1;
	}
	if (result.p[1] > 1) {
		result.p[1] = 1;
	}
	if (result.p[2] > 1) {
		result.p[2] = 1;
	}
	if (result.p[0] < 0) {
		result.p[0] = 0;
	}
	if (result.p[1] < 0) {
		result.p[1] = 0;
	}
	if (result.p[2] < 0) {
		result.p[2] = 0;
	}
    return result;
}

vector<Vec3Df> getVerticePoints(const vector<Vertex> &vertices) {
	vector<Vec3Df> points;
	for(int i=0; i<vertices.size(); ++i){
        points.push_back(vertices[i].p);
	}
	return points;
}

bool intersectionPoint(const Vec3Df &origin, const Vec3Df &dest, const vector<Vec3Df> &vertices, const Triangle &triangle, Vec3Df& result) {
	// Based on Moller-Trumbore intersection algorithm
	Vec3Df e0 = vertices[triangle.v[1]] - vertices[triangle.v[0]];
	Vec3Df e1 = vertices[triangle.v[2]] - vertices[triangle.v[0]];

	Vec3Df direction = dest - origin;
	direction.normalize();

	Vec3Df p = Vec3Df::crossProduct(direction, e1);

	float det = Vec3Df::dotProduct(e0, p);

	if (fabs(det) < 0.0000001) {
		return false;
	}
	float invDet = 1.f / det;

	Vec3Df t = origin - vertices[triangle.v[0]];

	float u = Vec3Df::dotProduct(t, p) * invDet;

	if (u < 0 || u > 1) {
		return false;
	}

	Vec3Df q = Vec3Df::crossProduct(t, e0);

	float v = Vec3Df::dotProduct(direction, q) * invDet;

	if (v < 0 || u + v > 1) {
		return false;
	}

	float a = Vec3Df::dotProduct(e1, q) * invDet;
	result = a * direction + origin;
	return a > 0.0000001;
}

void yourDebugDraw()
{
	float lightPosition[4] = {sunVector[0], sunVector[1], sunVector[2], 0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	rgb sunColor = sunVectorToRgb(sunVector);

	float lightColor[4] = {sunColor.r, sunColor.g, sunColor.b, 0.5};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);

	float specularColor[4] = {1, 1, 1, 0.5};
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);

	//draw open gl debug stuff
	//this function is called every frame

	//let's draw the mesh
	MyMesh.draw();

	//let's draw the lights in the scene as points
	glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
	glDisable(GL_LIGHTING);
	glColor3f(1,1,1);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (int i=0;i<MyLightPositions.size();++i)
		glVertex3fv(MyLightPositions[i].pointer());
	glEnd();
	glPopAttrib();//restore all GL attributes
	//The Attrib commands maintain the state.
	//e.g., even though inside the two calls, we set
	//the color to white, it will be reset to the previous
	//state after the pop.


	//as an example: we draw the test ray, which is set by the keyboard function
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(0,1,1);
	glVertex3f(testRayOrigin[0], testRayOrigin[1], testRayOrigin[2]);
	glColor3f(0,0,1);
	glVertex3f(testRayDestination[0], testRayDestination[1], testRayDestination[2]);
	glEnd();
	glPointSize(10);
	glBegin(GL_POINTS);
	glVertex3fv(MyLightPositions[0].pointer());
	glEnd();
	glPopAttrib();

	//draw whatever else you want...
	////glutSolidSphere(1,10,10);
	////allows you to draw a sphere at the origin.
	////using a glTranslate, it can be shifted to whereever you want
	////if you produce a sphere renderer, this
	////triangulated sphere is nice for the preview
}

void drawBox(BoundingBox box) {
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_TRIANGLES);
	glColor3f(1, 1, 1);
	vector<Vec3Df> boxVertices = box.getVertices();
	vector<unsigned int> boxIndices = box.getDrawingIndices();
	for(vector<unsigned int>::iterator it = boxIndices.begin(); it != boxIndices.end(); ++it) {
		int index = *it;
		Vec3Df vertex = boxVertices[index];
		glVertex3f(vertex[0], vertex[1], vertex[2]);
	}
	glEnd();
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}

const float ANGLE_STEP = (float) (2 * M_PI) / 72;
const hsv fullLight = {
	49,
	45,
	100
};

const hsv duskLight = {
	353,
	77,
	72
};

Vec3Df calculateSunVector() {
	return Vec3Df(cos(yawAngle) * sin(pitchAngle), cos(pitchAngle), -sin(yawAngle) * sin(pitchAngle)) * sunDist;
}

rgb sunVectorToRgb(Vec3Df sunVector) {
	sunVector.normalize();

	double angle = acos(Vec3Df::dotProduct(sunVector, Vec3Df(0, 1, 0)));

	double interpolation = angle / (M_PI * 0.5);

	double hueDiff = (duskLight.h - 360) - fullLight.h;
	double satDiff = duskLight.s - fullLight.s;
	double valueDiff = duskLight.v - fullLight.v;

	hsv currentLight = {
			(int) ((fullLight.h + (hueDiff * interpolation)) + 360) % 360,
			(fullLight.s + (satDiff * interpolation)) / 100,
			(fullLight.v + (valueDiff * interpolation)) / 100
	};
	return hsv2rgb(currentLight);
}

//yourKeyboardFunc is used to deal with keyboard input.
//t is the character that was pressed
//x,y is the mouse position in pixels
//rayOrigin, rayDestination is the ray that is going in the view direction UNDERNEATH your mouse position.
//
//A few keys are already reserved:
//'L' adds a light positioned at the camera location to the MyLightPositions vector
//'l' modifies the last added light to the current
//    camera position (by default, there is only one light, so move it with l)
//    ATTENTION These lights do NOT affect the real-time rendering.
//    You should use them for the raytracing.
//'r' calls the function performRaytracing on EVERY pixel, using the correct associated ray.
//    It then stores the result in an image "result.ppm".
//    Initially, this function is fast (performRaytracing simply returns
//    the target of the ray - see the code above), but once you replaced
//    this function and raytracing is in place, it might take a
//    while to complete...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination)
{

	//here, as an example, I use the ray to fill in the values for my upper global ray variable
	//I use these variables in the debugDraw function to draw the corresponding ray.
	//try it: Press a key, move the camera, see the ray that was launched as a line.
	testRayOrigin=rayOrigin;
	testRayDestination=rayDestination;

	// do here, whatever you want with the keyboard input t.
	if (t == 'w') {
		pitchAngle += ANGLE_STEP;
        calculateSun();
	} else if (t == 's') {
		pitchAngle -= ANGLE_STEP;
        calculateSun();
	} else if (t == 'a') {
		yawAngle += ANGLE_STEP;
        calculateSun();
	} else if (t == 'd') {
		yawAngle -= ANGLE_STEP;
        calculateSun();
	}
	//...

	cout<<t<<" pressed! The mouse was in location "<<x<<","<<y<<"!"<<endl;
}

Vec3Df intersectionWithPlane(const Vec3Df & planeNormal, Vec3Df & planePoint)
{
	Vec3Df dir = testRayDestination;
	Vec3Df origin = testRayOrigin;
	dir.normalize();
	origin.normalize();
	planePoint.normalize();

	float t = Vec3Df::dotProduct((planePoint - origin), planeNormal) / Vec3Df::dotProduct(dir, planeNormal);
	Vec3Df res = origin + t*dir;
	return res;
}

bool intersectionWithSphere(const Vec3Df & rayOrigin, const Vec3Df & rayDir, const Vec3Df sphereOrigin, const float & radius)
{
	float a = rayOrigin[0] - sphereOrigin[0];
	float b = rayOrigin[1] - sphereOrigin[1];
	float c = rayOrigin[2] - sphereOrigin[2];

	float A = (rayDir[0] * rayDir[0]) + (rayDir[1] * rayDir[1]) + (rayDir[2] * rayDir[2]);
	float B = 2 * ((rayDir[0] * a) + (rayDir[1] * b) + (rayDir[2] * c));
	float C = (a*a) + (b*b) + (c*c) - (radius*radius);
	
	float discriminant = (B*B) - (4 * A*C);
	if (discriminant >= 0) {
		return true;
	}
	else {
		return false;
	}

	/*
	Vec3Df intersectionP0;
	Vec3Df intersectionP1;
	float t0 = (-B + sqrtf(discriminant)) / (2 * A);
	float t1 = (-B - sqrtf(discriminant)) / (2 * A);
	
	//If the ray has one intersection point the two intersection points are the same
	//If the ray has two intersection points the two intersection points are different
	//If the ray has no intersection points the intersection points are null;
	if (discriminant >= 0) {
		intersectionP0 = rayOrigin + t0*(rayDir - rayOrigin);
		intersectionP1 = rayOrigin + t1*(rayDir - rayOrigin);
	}
	*/
}
