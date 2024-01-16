#include "Point.h"
#include <iostream>
#include <iomanip>
#include <unsupported/Eigen/MatrixFunctions> // for matrix.log()

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

bool Point::SetTensorLieDirect() const
{
	if (this->tensor == nullptr) return false;
	tensor->UpdateLieDirect(position);
	return true;
}

bool Point::SetTensorLieIndirect() const
{
	if (this->tensor == nullptr) return false;
	tensor->UpdateLieIndirect(position);
	return true;
}

bool Point::SetTensorLieGong() const
{
	if (this->tensor == nullptr) return false;
	tensor->UpdateLieGong(position);
	return true;
}

bool Point::SetTensorLieCalvo() const
{
	if (this->tensor == nullptr) return false;
	tensor->UpdateLieCalvo(position);
	return true;
}

bool Point::SetTensorLieLovric() const
{
	if (this->tensor == nullptr) return false;
	tensor->UpdateLieLovric(position);
	return true;
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

const Eigen::Matrix3d* Point::GetTensorMatrix() const
{
	if (this->tensor == nullptr) return nullptr;
	return &tensor->GetTensorMatrix();
}

const Eigen::Vector3d* Point::GetTensorEigenValues() const
{
	if (this->tensor == nullptr) return nullptr;
	return &tensor->GetEigenValues();
}

const Eigen::Vector3d* Point::GetTensorJValues() const
{
	if (this->tensor == nullptr) return nullptr;
	return &tensor->GetJValues();
}

const Eigen::Matrix4d* Point::GetLieMatrix() const
{
	if (this->tensor == nullptr) return nullptr;
	return &tensor->GetLieMatrix();
}

const Eigen::Matrix3d* Point::GetTensorEigenVectors() const
{
	if (this->tensor == nullptr) return nullptr;
	return &tensor->GetEigenVectors();
}

double Point::EuclideanDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	return (p1->GetPosition() - p2->GetPosition()).norm();
}

double Point::PureCTSF_TensorDistance(const Point* p1, const Point* p2)
{
	const Eigen::Vector3d* eigenValues1 = p1->GetTensorEigenValues();
	const Eigen::Vector3d* eigenValues2 = p2->GetTensorEigenValues();
	return (*eigenValues1 - *eigenValues2).squaredNorm();
}

double Point::CTSF_TensorDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	//SQR((t1.eigenValues(0) - t2.eigenValues(0))) + SQR((t1.eigenValues(1) - t2.eigenValues(1))) + SQR((t1.eigenValues(2) - t2.eigenValues(2)));
	const Eigen::Vector3d *eigenValues1 = p1->GetTensorEigenValues();
	const Eigen::Vector3d *eigenValues2 = p2->GetTensorEigenValues();
	return EuclideanDistance(p1, p2, weight) + weight* (*eigenValues1 - *eigenValues2).squaredNorm();
}

double Point::JDiff_TensorDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	const Eigen::Vector3d* jValues1 = p1->GetTensorJValues();
	const Eigen::Vector3d* jValues2 = p2->GetTensorJValues();
	return EuclideanDistance(p1, p2, weight) + weight * (*jValues1 - *jValues2).squaredNorm();
}

double Point::LieDirectDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	const Eigen::Matrix4d* lie1 = p1->GetLieMatrix();
	const Eigen::Matrix4d* lie2 = p2->GetLieMatrix();
	if (!lie1)
	{
		std::cout << "Point::LieDirectDistance: !lie1\n";
		exit(-1);
	}
	if (!lie2) 
	{	
		std::cout << "Point::LieDirectDistance: !lie2\n";
		exit(-1);
	}
	if (verbose)
	{
		std::cout << std::setprecision(15) << "log1 " << lie1->norm() << "\n" << *lie1 << std::endl;
		std::cout << "log2 " << lie2->norm() << "\n" << *lie2 << std::endl;
		std::cout << "sub " << (*lie1 - *lie2).norm() << "\n" << (*lie1 - *lie2) << std::endl;
		std::cout << "W*sub.norm() " << weight * (*lie1 - *lie2).norm() << std::endl;
	}
	return weight * (*lie1 - *lie2).norm(); // For matrices, norm() is the Frobenius norm.
}

double Point::LieIndirectDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	const Eigen::Matrix4d* lie1 = p1->GetLieMatrix();
	const Eigen::Matrix4d* lie2 = p2->GetLieMatrix();
	if (!lie1)
	{
		std::cout << "Point::LieIndirectDistance: !lie1\n";
		exit(-1);
	}
	if (!lie2)
	{
		std::cout << "Point::LieIndirectDistance: !lie2\n";
		exit(-1);
	}
	if (verbose)
	{
		std::cout << std::setprecision(15) << "log1 " << lie1->norm() << "\n" << *lie1 << std::endl;
		std::cout << "log2 " << lie2->norm() << "\n" << *lie2 << std::endl;
		std::cout << "sub " << (*lie1 - *lie2).norm() << "\n" << (*lie1 - *lie2) << std::endl;
		std::cout << "W*sub.norm() " << weight * (*lie1 - *lie2).norm() << std::endl;
	}
	return weight * (*lie1 - *lie2).norm(); // For matrices, norm() is the Frobenius norm.
}

double Point::LieGongDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	const Eigen::Matrix4d* lie1 = p1->GetLieMatrix();
	const Eigen::Matrix4d* lie2 = p2->GetLieMatrix();
	if (!lie1)
	{
		std::cout << "Point::LieGongDistance: !lie1\n";
		exit(-1);
	}
	if (!lie2)
	{
		std::cout << "Point::LieGongDistance: !lie2\n";
		exit(-1);
	}
	Eigen::Matrix4d aux = lie1->inverse();
	aux = aux * (*lie2);
	aux = aux.log();
	return weight * aux.norm(); // For matrices, norm() is the Frobenius norm.
}

double Point::LieCalvoDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	const Eigen::Matrix4d* lie1 = p1->GetLieMatrix();
	const Eigen::Matrix4d* lie2 = p2->GetLieMatrix();
	if (!lie1)
	{
		std::cout << "Point::LieCalvoDistance: !lie1\n";
		exit(-1);
	}
	if (!lie2)
	{
		std::cout << "Point::LieCalvoDistance: !lie2\n";
		exit(-1);
	}
	Eigen::Matrix4d aux = lie1->inverse();
	aux = aux * (*lie2);
	aux = aux.log();
	return weight * aux.norm(); // For matrices, norm() is the Frobenius norm.
}

double Point::LieLovricDistance(const Point* p1, const Point* p2, const double weight, const bool verbose)
{
	const Eigen::Matrix4d* lie1 = p1->GetLieMatrix();
	const Eigen::Matrix4d* lie2 = p2->GetLieMatrix();
	if (!lie1)
	{
		std::cout << "Point::LieLovricDistance: !lie1\n";
		exit(-1);
	}
	if (!lie2)
	{
		std::cout << "Point::LieLovricDistance: !lie2\n";
		exit(-1);
	}
	Eigen::Matrix4d aux = lie1->inverse();
	aux = aux * (*lie2);
	aux = aux.log();
	return weight * aux.norm(); // For matrices, norm() is the Frobenius norm.
}

