/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream> 
#include "util.h"
#include <cstdlib>

#include <iomanip>
#include <sstream>
#include <string>

//global variable for screen size



Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;

	
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {

			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
}

void Raytracer::textureMapping(Ray3D& ray) {
//1
	int index;	
	Point3D inter = ray.intersection.point;
	Point3D centre = ray.intersection.centre;
	double sub = inter[2] - centre[2];
	//should divide by radius here,but we have unit sphere
	while(sub > 1){
		sub = sub/10;
	}
	while(sub < -1){
		sub = sub/10;
	}
	double t = (PI - acos(sub))/(double) PI;
	double p = atan2(inter[1] - centre[1], inter[0]-centre[0]);
	double u = (p / (double) (2 * PI)) + 0.5;
	
	u = u * width;
	t = t * height;
	index = floor(t) * width + floor(u);
	//std::cout << "---------------------------------acos: "<< acos(inter[2] - centre[2])<<"\n";
	//std::cout << "---------------------------------inter: "<< inter[2] - centre[2] <<"\n";
	//std::cout << "---------------------------------here2: "<< index<<"\n";;

	
	ray.col[0] += (texture_rbuffer[index])/(double)255;
	//std::cout << "---------------------------------here3 :"<< index<<"\n";
	ray.col[1] += (texture_gbuffer[index])/(double)255;
	ray.col[2] += (texture_bbuffer[index])/(double)255;
	

	
}


void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
	for (;;) {
		if (curLight == NULL)
			break;
		
		//object lays between the intersection point and the light
		bool shadowed = false;
		Vector3D rayDir = curLight->light->get_position()- ray.intersection.point;
		//Calculate the distance between this intersection point and the light
		double distance = sqrt(pow(rayDir[0], 2) + pow(rayDir[1], 2) + pow(rayDir[2], 2));

		Ray3D ShadowRay;
		ShadowRay.dir = rayDir;
		ShadowRay.dir.normalize();
		ShadowRay.origin = ray.intersection.point + EPSILON * ShadowRay.dir;

		//Test the intersection of this shadowray
		traverseScene(_root, ShadowRay);

		// check if object intersect with shadowRay
		if (!ShadowRay.intersection.none&& ShadowRay.intersection.mat->transparency == 0) {
			Vector3D TRaytoLight = curLight->light->get_position()
					- ShadowRay.intersection.point;
			double TRaydistance = sqrt(
					pow(TRaytoLight[0], 2) + pow(TRaytoLight[1], 2)
							+ pow(TRaytoLight[2], 2));
			//if the object is inbetween the intersection point and the light
			if (distance - TRaydistance > 0.01)
				ray.shadowed = true;
		}
		
		switch (_mode){
				case(Sig):
					curLight->light->shadeSig(ray);
					break;
				case(Phong):
					curLight->light->shade(ray);
					break;
				case(Diffuse):
					curLight->light->shadeDiffuse(ray);
					break;
				case(Other):
					//default is phong
					curLight->light->shade(ray);
					break;
			}
			

		curLight = curLight->next;
	}
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j]= 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray) {
	Colour col(0.0, 0.0, 0.0); 
	Colour refrcol;
	traverseScene(_root, ray); 
	//std::cout << "markone"<< col[0] << " def.\n";
	
	Intersection intersection = ray.intersection;
	// Don't bother shading if the ray didn't hit anything.
	if (!intersection.none && (ray.depth < maxDepth)) {
		
		computeShading(ray);
		col = ray.col;
		
		//Reflection implementation
		if ( ray.intersection.mat->reflectivity > 0 && ray.nbounces < MAX_BOUNCES) {
			Ray3D reflect;
			//Increment the bounce value
			reflect.nbounces = ray.nbounces + 1;
			reflect.origin = ray.intersection.point;
			Vector3D norm = ray.dir;
			norm.normalize();
			//direction of reflected ray
			reflect.dir = norm- 2 * ray.intersection.normal.dot(norm)* ray.intersection.normal;
			reflect.dir.normalize();
			//Recalculate the reflect ray origin
			reflect.origin = reflect.origin + EPSILON * reflect.dir;

			Colour colReflected;
			
			//mirror reflection
			reflect.depth = ray.depth + 1;
			colReflected = shadeRay(reflect);
			col = col + (ray.intersection.mat->reflectivity) * colReflected;
			col.clamp();
		}
		
		
		double in, out;
		//REFRACTION
		
		if(_mode == Other && ray.refraction %2 != 0){
			out = ray.intersection.mat->refra_index;
			in  = 1;
		}else{
			out = 1;
			in  = ray.intersection.mat->refra_index;
		}
		
		double ratio = out/in;
		Vector3D direct = -ray.dir;
		direct.normalize();
		double v = ray.intersection.normal.dot(direct)*ray.intersection.normal.dot(direct);
		v = 1.0 - ratio * ratio *(1.0 - v);
		
		if(v >= 0){
		Ray3D refr;
		Vector3D nor = ray.intersection.normal;
		Vector3D refrDir = (ratio * nor.dot(direct) - sqrt(v)) * nor;
		refrDir = refrDir - ratio * direct;
		refrDir.normalize();
		refr.origin =  ray.intersection.point + EPSILON * ray.dir;
		refr.dir = refrDir;
		refr.refraction = ray.refraction + 1;
		refr.depth = ray.depth + 1;
		refrcol = pow(0.7, ray.depth) * shadeRay(refr);

		}else{
			
			refrcol = Colour(0.0, 0.0,0.0);
		}
		
		
	} 
	col = refrcol + col;
	col.clamp();
	return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);
	
	int k = 0;

	// Construct a ray for each pixel.
	Colour col(0, 0, 0);
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			// Sets up ray origin and direction in view space, 
			// image plane is at z = -1.
			Point3D origin(0, 0, 0);
			Point3D imagePlane;
			imagePlane[2] = -1;

			// TODO: Convert ray to world space and call 
			// shadeRay(ray) to generate pixel colour. 	

			Ray3D ray;
			ray.origin = viewToWorld * origin;
			ray.depth = 0;
			


			imagePlane[0] = (-double(width) / 2 + 0.5 + j) / factor;
			imagePlane[1] = (-double(height) / 2 + 0.5 + i) / factor;
			
			//imagePlane[2] = -1;
			
			
			ray.dir = viewToWorld* Point3D(imagePlane[0], imagePlane[1], imagePlane[2])	- ray.origin;
			
			ray.dir.normalize();
			col = shadeRay(ray);
			
			_rbuffer[i * width + j] = int(col[0] * 255);	
				
			_gbuffer[i * width + j] = int(col[1] * 255);
			_bbuffer[i * width + j] = int(col[2] * 255);
		}
		//if (i%40 == 0){
		//std::cout << "Row "<< i << " shaded.\n";}
	}

	flushPixelBuffer(fileName);
	printf("%s is rendered.\n", fileName);
}

void Raytracer::setMode(int mode){
	_mode = mode;
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320; 
	int height = 240; 

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 1);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 60;
	Point3D eye2(4, 2, 1);
	Vector3D view2(-4, -2, -6);

	// Defines a material for shading.
	Material gold(Colour(0.3, 0.3, 0.02), Colour(0.75164, 0.60648, 0.22648),
			Colour(0.628281, 0.555802, 0.366065), 51.2, 0.1, 0, 0, false, 0.0);
	Material jade(Colour(0.135, 0.2225, 0.1575), Colour(0.54, 0.89, 0.63),
			Colour(0.316228, 0.316228, 0.316228), 13 , 0.1, 0, 0, false, 0.34);
	Material glass(Colour(0, 0, 0), Colour(0, 0, 0), Colour(1, 1, 1), 80.0, 0.1,
			1, 0, false, 1.3);
	Material silver(Colour(0, 0, 0),
			Colour(0.2775, 0.2775, 0.2775),
			Colour(0.773911, 0.773911, 0.773911), 70, 0.1, 0, 0,false, 1.6);



	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), 
				Colour(1, 1, 1) ) );

	// Add a unit square into the scene with material mat.
	SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &gold );
	SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );
	//SceneDagNode* cylinder = raytracer.addObject( new UnitCylinder(), &silver );
	
	// Apply some transformations to the unit square.
	double factor1[3] = { 1.0, 1.5, 1.0 };
	double factor2[3] = { 8.0, 8.0, 8.0 };
	double factor3[3]= { 1, 1, 1 };
	raytracer.translate(sphere, Vector3D(0, 0, -3));	
	raytracer.rotate(sphere, 'x', -35); 
	raytracer.rotate(sphere, 'z', 25); 
	raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

	raytracer.translate(plane, Vector3D(0, 0, -7));	
	raytracer.rotate(plane, 'z', 45); 
	raytracer.scale(plane, Point3D(0, 0, 0), factor2);
	
	//raytracer.translate(cylinder, Vector3D(2, 1, 0));
	//raytracer.scale(cylinder, Point3D(0, 0, 0), factor3);

	/*********************render************************/
	//you can set mode to Sig, Diffuse or Phong here
	//for part 2 set mode to Other and enable flag in raytracer.h
	raytracer.setMode(Raytracer::Sig);
	raytracer.render(width, height, eye, view, up, fov, "sig1.bmp");
	std::cout << "---------------------------------sig1.bmp rendered\n";

	raytracer.render(width, height, eye2, view2, up, fov, "sig2.bmp");
	std::cout << "---------------------------------sig2.bmp rendered\n";

	

	raytracer.setMode(Raytracer::Diffuse);
	raytracer.render(width, height, eye, view, up, fov, "Diffuse1.bmp");
	std::cout << "---------------------------------diffuse1.bmp rendered\n";

	raytracer.render(width, height, eye2, view2, up, fov, "Diffuse2.bmp");
	std::cout << "---------------------------------diffuse2.bmp rendered\n";
	
	raytracer.setMode(Raytracer::Phong);
	raytracer.render(width, height, eye, view, up, fov, "Phong1.bmp");
	std::cout << "---------------------------------phong1.bmp rendered\n";

	raytracer.render(width, height, eye2, view2, up, fov, "Phong2.bmp");
	std::cout << "---------------------------------phong2.bmp rendered\n";
	
	return 0;
}

