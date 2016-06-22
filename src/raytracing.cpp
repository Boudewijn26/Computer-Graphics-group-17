#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include <float.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "raytracing.h"


//temporary variables
//these are only used to illustrate
//a simple debug drawing. A ray
Vec3Df testRayOrigin;
Vec3Df testRayDestination;

float pitchAngle = 0;
float yawAngle = 0;

std::vector<BoundingBox> boxes;
std::vector<Vec3Df> meshPoints;

void drawBox(BoundingBox box);

bool isTriangleHit(const Vec3Df &origin, const Vec3Df &dest, const Vec3Df &v0, const Vec3Df &v1, const Vec3Df &v2);

Vec3Df calculateSunVector() ;

rgb sunVectorToRgb(Vec3Df sunVector) ;

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
    MyMesh.loadMesh("models/dodgeColorTest.obj", true);
	MyMesh.computeVertexNormals();
    meshPoints = getVerticePoints(MyMesh.vertices);
	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);

	BoundingBox main = BoundingBox(MyMesh);
	boxes = main.split(2000);
	printf("Calculated bounding box with %d boxes", boxes.size());
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	Vec3Df result;
	if (trace(origin, dest, 0, result)) return result;
	return Vec3Df(0, 0, 0);
}

bool trace(const Vec3Df & origin, const Vec3Df & dest, int level, Vec3Df& result) {
	Intersection intersection;
	Vec3Df intersect;
	bool foundBox = false;
    for(int i=0; i<boxes.size(); ++i) {
		if (foundBox) {
			break;
		}
        std::vector<Triangle> triangles = boxes[i].getBoundingTriangles();
        std::vector<Vec3Df> vertices = boxes[i].getVertices();
        for(int j=0; j<=triangles.size(); ++j){
			if (j == triangles.size()) break;
            if (intersectionPoint(origin, dest, vertices, triangles[j], intersect)) {
				foundBox = true;
				break;
            }
        }

    }
	if (!foundBox) {
		return false;
	}
    for(int i=0; i<MyMesh.triangles.size(); ++i) {
        Triangle triangle = MyMesh.triangles[i];
        if (intersectionPoint(origin, dest, meshPoints, triangle, intersect)) {
            std::cout<<"Intersection found! at triangle:"<<i<<std::endl;
			float distance = (intersect - origin).getLength();
            std::cout<<"With distance:"<<distance<<" and color:"<<shade(intersection, level)<<std::endl;
			if (intersection.distance > distance) {
				intersection.distance = distance;
				intersection.index = i;
				intersection.intersect = intersect;
			}
        }
    }
	if (intersection.index == -1) return false;

	result = shade(intersection, level);

	return true;
}

Vec3Df shade(Intersection intersection, int level) {
    Vec3Df refl = Vec3Df(0,0,0);
	Vec3Df refr = Vec3Df(0,0,0);
    Vec3Df direct;
    for(int i=0; i<MyLightPositions.size(); i++){
		unsigned int triMat = MyMesh.triangleMaterials.at(intersection.index);
		direct = MyMesh.materials.at(triMat).Kd();
    }
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
    return direct;
}

std::vector<Vec3Df> getVerticePoints(const std::vector<Vertex> &vertices) {
	std::vector<Vec3Df> points;
	for(int i=0; i<vertices.size(); ++i){
        points.push_back(vertices[i].p);
	}
	return points;
}

bool intersectionPoint(const Vec3Df &origin, const Vec3Df &dest, const std::vector<Vec3Df> &vertices, const Triangle &triangle, Vec3Df& result) {

    //Find vectors for two edges sharing V1
  SUB(e1, V2, V1);
  SUB(e2, V3, V1);
  //Begin calculating determinant - also used to calculate u parameter
  CROSS(P, D, e2);
  //if determinant is near zero, ray lies in plane of triangle
  det = DOT(e1, P);
  //NOT CULLING
  if(det > -EPSILON && det < EPSILON) return 0;
  inv_det = 1.f / det;

  //calculate distance from V1 to ray origin
  SUB(T, O, V1);

  //Calculate u parameter and test bound
  u = DOT(T, P) * inv_det;
  //The intersection lies outside of the triangle
  if(u < 0.f || u > 1.f) return 0;

  //Prepare to test v parameter
  CROSS(Q, T, e1);

  //Calculate V parameter and test bound
  v = DOT(D, Q) * inv_det;
  //The intersection lies outside of the triangle
  if(v < 0.f || u + v  > 1.f) return 0;

  t = DOT(e2, Q) * inv_det;

  if(t > EPSILON) { //ray intersection
    *out = t;
    return 1;
  }

  // No hit, no win
  return 0;


	Vec3Df q = dest - origin;
	Vec3Df a = vertices[triangle.v[0]] - origin;
	Vec3Df b = vertices[triangle.v[1]] - origin;
	Vec3Df c = vertices[triangle.v[2]] - origin;

    Vec3Df e1 = b-a;
    Vec3Df e2 = c-a;

    Vec3Df P = Vec3Df::crossProduct(q, e2);
    float det = Vec3Df::dotProduct(e1,P);
    if(det > -FLT_EPSILON && det < FLT_EPSILON) return false;
    inv_det = 1.f / det;

    Vec3Df T = origin - a;

	float u = Vec3Df::dotProduct(T, P)*inv_det;
	if (u < 0.f || u > 1.f) return false;
	Vec3Df Q = Vec3Df::crossProduct(T, e1);

	float v = Vec3Df::dotProduct(q, Q);
	if (v < 0.f || u + v > 1.f) return false;

	float t = Vec3Df::dotProduct(e2, Q) * inv_det;
	if (t > FLT_EPSILON){
        //float d = 1.0f / (u + v + w);
        result = a + u*c + v*b;
	}

	return false;
}

void yourDebugDraw()
{
	Vec3Df sunVector = calculateSunVector();

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

	for (std::vector<BoundingBox>::iterator it = boxes.begin(); it != boxes.end(); ++it) {
		BoundingBox box = *it;
		drawBox(box);
	}
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
	std::vector<Vec3Df> boxVertices = box.getVertices();
	std::vector<unsigned int> boxIndices = box.getDrawingIndices();
	for(std::vector<unsigned int>::iterator it = boxIndices.begin(); it != boxIndices.end(); ++it) {
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
	return Vec3Df(cos(yawAngle) * sin(pitchAngle), cos(pitchAngle), -sin(yawAngle) * sin(pitchAngle));
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
	} else if (t == 's') {
		pitchAngle -= ANGLE_STEP;
	} else if (t == 'a') {
		yawAngle += ANGLE_STEP;
	} else if (t == 'd') {
		yawAngle -= ANGLE_STEP;
	}
	//...
	Vec3Df resulting;
    bool trace1 = trace(rayOrigin, rayDestination, 0, resulting);
    std::cout<<"Hit: "<<trace1<<", color: "<<resulting.p[0]<<" "<<resulting.p[1]<<" "<<resulting.p[2]<<std::endl;
	std::cout<<t<<" pressed! The mouse was in location "<<x<<","<<y<<"!"<<std::endl;
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

Vec3Df intersectionWithSphere(const Vec3Df & rayOrigin, const Vec3Df & rayDest, const float & radius)
{
	Vec3Df p1 = rayOrigin;
	Vec3Df p2 = rayDest;

	float A = p2[0] - p1[0];
	float B = p2[1] - p1[1];
	float C = p2[2] - p1[2];

	float a = (A*A) + (B*B) + (C*C);
	float b = 2 * ((A*p1[0]) + (B*p1[1]) + (C*p1[2]));
	float c = (p1[0] * p1[0]) + (p1[1] * p1[1]) + (p1[2] * p1[2]) - (radius*radius);

	float t = (-b + sqrtf((b*b) - (4 * a*c))) / (2 * a);

	Vec3Df res = p1 + t*(p2 - p1);
	return res;
}
