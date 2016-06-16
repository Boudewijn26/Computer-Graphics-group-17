#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include <GL/gl.h>
#include <cfloat>
#include "raytracing.h"


//temporary variables
//these are only used to illustrate
//a simple debug drawing. A ray
Vec3Df testRayOrigin;
Vec3Df testRayDestination;

std::vector<BoundingBox> boxes;
std::vector<Triangle> boundingTriangles;

void drawBox(BoundingBox box);

bool isTriangleHit(const Vec3Df &origin, const Vec3Df &dest, const Vec3Df &v0, const Vec3Df &v1, const Vec3Df &v2);

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
    MyMesh.loadMesh("models/cube.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);

	BoundingBox main = BoundingBox(MyMesh);
	boxes = main.split(500);
	boundingTriangles = main.getBoundingTriangles();
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
    for(int i=0; i<boundingTriangles.size(); ++i) {
        Triangle triangle = boundingTriangles[i];
        if (!intersectionPoint(origin, dest, triangle, intersect)) {
            return false;
        }
    }
    for(int i=0; i<MyMesh.triangles.size(); ++i) {
        Triangle triangle = MyMesh.triangles[i];
        if (intersectionPoint(origin, dest, triangle, intersect)) {
			float distance = (intersect - origin).getLength();
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

bool intersectionPoint(const Vec3Df &origin, const Vec3Df &dest, const Triangle &triangle, Vec3Df& result) {
	Vec3Df q = dest - origin;
	Vec3Df a = MyMesh.vertices[triangle.v[0]].p - origin;
	Vec3Df b = MyMesh.vertices[triangle.v[1]].p - origin;
	Vec3Df c = MyMesh.vertices[triangle.v[2]].p - origin;

	float u = Vec3Df::dotProduct(b, Vec3Df::crossProduct(q, c));
	if (u < FLT_EPSILON) return false;
	float v = Vec3Df::dotProduct(a, -Vec3Df::crossProduct(q, c));
	if (v < FLT_EPSILON) return false;
	float w = Vec3Df::dotProduct(q, Vec3Df::crossProduct(b, a));
	if (w < FLT_EPSILON) return false;

	float d = 1.0f / (u + v + w);

	result = Vec3Df(u*d, v*d, w*d);

	return true;
}

void yourDebugDraw()
{
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

	//...


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
