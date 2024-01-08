#ifndef POINT_H
#define POINT_H

#include "Tensor.h"


class Point
{
    public:
		enum class DISTANCE_TYPE { EUCLIDEAN, CTSF};

        Point(double x, double y, double z);
        virtual ~Point();

		const Eigen::Vector3d& GetPosition() const;
		const Eigen::Vector3d& GetColor() const;
		bool IsRemoved();
		void SetAsRemoved(bool removed);
		void SetColor(const Eigen::Vector3d& color);
		void Translate(const Eigen::Vector3d &translation);
		void Rotate(const Eigen::Affine3d &rotation);		

		void SetTensor(const Eigen::Matrix3d &tensorMatrix);
		Eigen::Vector3d GetNormal() const;

		double GetTensorPlanarCoefficient() const;
		Eigen::Matrix3d* GetTensorEigenVectors() const;
		// temporary
		Eigen::Matrix3d* GetTensorMatrix() const;
		Eigen::Vector3d* GetTensorEigenValues() const;

		static double Distance(const Point *p1, const Point *p2, const DISTANCE_TYPE t);

    protected:

    private:
		Eigen::Vector3d position;
		Eigen::Vector3d color;
		Eigen::Vector3d normal;
		bool removed = false;

		Tensor* tensor = nullptr;


};

#endif // POINT_H