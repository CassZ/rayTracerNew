/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"
//#include "raytracer.h"
extern int width;
extern int height;
extern unsigned char* texture_rbuffer;
extern unsigned char* texture_gbuffer;
extern unsigned char* texture_bbuffer;
const double PI = 3.14159265;

//phong
void PointLight::shade( Ray3D& ray ) {
	// TODO: implement this function to fill in values for ray.col 
	// using phong shading.  Make sure your vectors are normalized, and
	// clamp colour values to 1.0.
	//
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function. 
	Vector3D light = get_position() - ray.intersection.point,
	         eye = -ray.dir,
		     normal = ray.intersection.normal;
	Material *material = ray.intersection.mat;


	light.normalize();
	eye.normalize();
	
	double diffuse = normal.dot(light);
	if(diffuse < 0){
		diffuse = 0;
	}
	
	double specular = (2 * normal.dot(light) * normal - light).dot(eye);
	if(specular < 0){
		specular = 0;
	}
	
	specular = pow(specular, material->specular_exp);


	// phong 
	Colour colour = _col_ambient * material->ambient
		+ _col_diffuse * material->diffuse * Colour(diffuse, diffuse, diffuse)
		+ _col_specular * material->specular * Colour(specular, specular, specular);
	if(ray.intersection.mat->texture){
		

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
	

	
}else{
	
	ray.col = colour;
	}
	ray.col.clamp();
	return;
 }
 
void PointLight::shadeSig( Ray3D& ray ){
		Vector3D light = get_position() - ray.intersection.point,
	         eye = -ray.dir,
		     normal = ray.intersection.normal;
	Material *material = ray.intersection.mat;

	// sig 
	ray.col = material->diffuse;
	ray.col.clamp();
	return;	
}
/* void PointLight::shadeSig(Ray3D & ray){
	 Intersection intersectionPoint = ray.intersection;
	 ray.col = intersectionPoint.mat->diffuse;
	 ray.col.clamp();
 }*/
 
void PointLight::shadeDiffuse( Ray3D& ray ){
	Vector3D light = get_position() - ray.intersection.point, 
	eye = -ray.dir,
	normal = ray.intersection.normal;
	Material *material = ray.intersection.mat;

	light.normalize();
	eye.normalize();
	
	double diffuse = normal.dot(light), 
	specular = (2 * normal.dot(light) * normal - light).dot(eye);

	if(diffuse < 0){
		diffuse = 0;
	}
	if(specular < 0){
		specular = 0;
	}
	
	specular = pow(specular, material->specular_exp);

	// diffuse
	Colour colour = _col_ambient * material->ambient+ _col_diffuse * material->diffuse * Colour(diffuse, diffuse, diffuse);

	ray.col = colour;
	ray.col.clamp();
	return;
}
