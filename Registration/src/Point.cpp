#include "Point.h"

Point::Point(double x, double y, double z)
{
    //ctor
	this->position[0] = x;
	this->position[1] = y;
	this->position[2] = z;

	this->color[0] = 0.0;
	this->color[1] = 0.0;
	this->color[2] = 0.0;

	this->removed = false;
}

Point::~Point()
{
    //dtor
}

const Eigen::Vector3d& Point::getPosition() const
{
	return this->position;
}

const Eigen::Vector3d& Point::getColor() const
{
	return this->color;
}

bool Point::isRemoved()
{
	return this->removed;
}

void Point::setAsRemoved(bool removed)
{
	this->removed = removed;
}

void Point::setColor(const Eigen::Vector3d &color)
{
	this->color = color;
}

void Point::translate(const Eigen::Vector3d &translation)
{
	this->position += translation;
}

void Point::rotate(const Eigen::Affine3d &rotation)
{
	this->position = rotation*this->position;
}