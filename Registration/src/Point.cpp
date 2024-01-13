#include "Point.h"
#include <iostream>

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

const Eigen::Vector3d& Point::GetPosition() const
{
	return this->position;
}

const Eigen::Vector3d& Point::GetColor() const
{
	return this->color;
}

bool Point::IsRemoved()
{
	return this->removed;
}

void Point::SetAsRemoved(bool removed)
{
	this->removed = removed;
}

void Point::SetColor(const Eigen::Vector3d &color)
{
	this->color = color;
}

void Point::Translate(const Eigen::Vector3d &translation)
{
	this->position += translation;
}

void Point::Rotate(const Eigen::Affine3d &rotation)
{
	this->position = rotation*this->position;
}

void Point::SetTensor(const Eigen::Matrix3d& tensorMatrix)
{
	if (this->tensor == nullptr)
	{
		tensor = new Tensor();
	}

	tensor->Update(tensorMatrix);
}

Eigen::Vector3d Point::GetNormal() const
{
	if(this->tensor == nullptr) return Eigen::Vector3d(0.0,0.0,0.0);
	return tensor->GetEigenVectors().row(2).real();
}

double Point::GetTensorPlanarCoefficient() const
{
	if (this->tensor == nullptr) return 0.0;
	return this->tensor->GetPlanarCoefficient();
}

Eigen::Matrix3d* Point::GetTensorMatrix() const
{
	if (this->tensor == nullptr) return nullptr;
	Eigen::Matrix3d* m = new Eigen::Matrix3d();
	*m = tensor->GetTensorMatrix();
	return m;
}

Eigen::Vector3d* Point::GetTensorEigenValues() const
{
	if (this->tensor == nullptr) return nullptr;
	Eigen::Vector3d* v = new Eigen::Vector3d();
	*v = tensor->GetEigenValues();
	return v;
}

Eigen::Vector3d* Point::GetTensorJValues() const
{
	if (this->tensor == nullptr) return nullptr;
	Eigen::Vector3d* v = new Eigen::Vector3d();
	*v = tensor->GetJValues();
	return v;
}

Eigen::Matrix3d* Point::GetTensorEigenVectors() const
{
	if (this->tensor == nullptr) return nullptr;
	Eigen::Matrix3d* m = new Eigen::Matrix3d();
	*m = tensor->GetEigenVectors();
	return m;
}

double Point::EuclideanDistance(const Point* p1, const Point* p2, const double weight)
{
	return (p1->GetPosition() - p2->GetPosition()).norm();
}

double Point::PureCTSF_TensorDistance(const Point* p1, const Point* p2)
{
	const Eigen::Vector3d* eigenValues1 = p1->GetTensorEigenValues();
	const Eigen::Vector3d* eigenValues2 = p2->GetTensorEigenValues();
	return (*eigenValues1 - *eigenValues2).squaredNorm();
}
double Point::CTSF_TensorDistance(const Point* p1, const Point* p2, const double weight)
{
	//SQR((t1.eigenValues(0) - t2.eigenValues(0))) + SQR((t1.eigenValues(1) - t2.eigenValues(1))) + SQR((t1.eigenValues(2) - t2.eigenValues(2)));
	const Eigen::Vector3d *eigenValues1 = p1->GetTensorEigenValues();
	const Eigen::Vector3d *eigenValues2 = p2->GetTensorEigenValues();
	return EuclideanDistance(p1, p2, weight) + weight* (*eigenValues1 - *eigenValues2).squaredNorm();
}

double Point::JDiff_TensorDistance(const Point* p1, const Point* p2, const double weight)
{
	const Eigen::Vector3d* jValues1 = p1->GetTensorJValues();
	const Eigen::Vector3d* jValues2 = p2->GetTensorJValues();
	return EuclideanDistance(p1, p2, weight) + weight * (*jValues1 - *jValues2).squaredNorm();
}

/*double Point::Distance(const Point* p1, const Point* p2, const DISTANCE_TYPE t)
{
	if (t == DISTANCE_TYPE::EUCLIDEAN) return (p1->GetPosition()-p2->GetPosition()).norm();
	return 0.0;
}*/
