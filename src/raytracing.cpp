#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include "raytracing.h"


//temporary variables
//these are only used to illustrate
//a simple debug drawing. A ray
Vec3Df testRayOrigin;
Vec3Df testRayDestination;


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
    MyMesh.loadMesh("cube.obj", true);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	return Vec3Df(dest[0],dest[1],dest[2]);
}

Vec3Df trace(const Vec3Df & origin, const Vec3Df & dest)
{
    printf("begin");
    for(int i=0; i<MyMesh.triangles.size(); ++i)
        {
            Triangle triangle = MyMesh.triangles[i];
            if(calculateHit(origin, dest, triangle))
                printf("calculateHit: true");
        }
}

bool calculateHit(const Vec3Df & origin, const Vec3Df & dest, const Triangle & triangle)
{
    Vec3Df v0 = MyMesh.vertices[triangle.v[0]].p;
    Vec3Df v1 = MyMesh.vertices[triangle.v[1]].p;
    Vec3Df v2 = MyMesh.vertices[triangle.v[2]].p;

    // compute plane's normal
    Vec3Df v0v1 = v1 - v0;
    Vec3Df v0v2 = v2 - v0;
    // no need to normalize
    Vec3Df N = Vec3Df::crossProduct(v0v2, v0v1); // N
    float area2 = N.getLength();
    float t;
    float u;
    float v;

// Step 1: finding P

// check if ray and plane are parallel ?
    float NdotRayDirection = Vec3Df::dotProduct(dest, N);
    if (fabs(NdotRayDirection) < 0.0000001) // almost 0
    return false; // they are parallel so they don't intersect !

// compute d parameter using equation 2
    float d = Vec3Df::dotProduct(v0, N);

// compute t (equation 3)
    t = (Vec3Df::dotProduct(origin, N) + d) / NdotRayDirection;
// check if the triangle is in behind the ray
    if (t < 0) return false; // the triangle is behind

// compute the intersection point using equation 1
    Vec3Df P = origin + t * dest;

// Step 2: inside-outside test
    Vec3Df C; // vector perpendicular to triangle's plane

// edge 0
    Vec3Df edge0 = v1 - v0;
    Vec3Df vp0 = P - v0;
    C = Vec3Df::crossProduct(vp0, edge0);
    if (Vec3Df::dotProduct(C, N) < 0) return false; // P is on the right side

// edge 1
    Vec3Df edge1 = v2 - v1;
    Vec3Df vp1 = P - v1;
    C = Vec3Df::crossProduct(vp1, edge1);
    u = C.getLength() / area2;
    if (Vec3Df::dotProduct(C, N) < 0) return false; // P is on the right side

// edge 2
    Vec3Df edge2 = v0 - v2;
    Vec3Df vp2 = P - v2;
    C = Vec3Df::crossProduct(vp2, edge2);
    v = C.getLength() / area2;
    if (Vec3Df::dotProduct(C, N) < 0) return false; // P is on the right side;

return true; // this ray hits the triangle
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
