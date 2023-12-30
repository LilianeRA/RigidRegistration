#include "Graphics.h"
//#include "GL/glew.h"
#include <iostream>

Graphics::Graphics(double radius)
{
    //ctor
	this->radius = radius;
    this->vertices.clear();
}

Graphics::~Graphics()
{
    //dtor
    this->vertices.clear();
}


void Graphics::setPoints(std::vector<Point*> vertices)
{
	this->vertices.clear();
	this->vertices = vertices;
}

void Graphics::setRadius(double radius)
{
	this->radius = radius;
}

void Graphics::draw()
{
	//if(vertices.size() == 0) return;
	
	/*double scale = 100.0;
	Eigen::Vector3d color = vertices.at(0)->getColor();
	glColor3f(color[0], color[1], color[2]);
	for(unsigned int i = 0; i < vertices.size(); i++)
	{ 
		Point *p = vertices.at(i);
		if(p->isRemoved()) continue;
		Eigen::Vector3d pos = p->getPosition();
		glPushMatrix();
		glScalef(scale, scale, scale);	
		glTranslatef(pos[0], pos[1], pos[2]);
		gluSphere(gluNewQuadric(),radius/scale,10,10);
		glPopMatrix();
	}*/
}