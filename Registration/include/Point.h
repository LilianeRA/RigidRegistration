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
		bool SetLieTensor();
		Eigen::Vector3d GetNormal() const;

		double GetTensorPlanarCoefficient() const;
		Eigen::Matrix3d* GetTensorEigenVectors() const;

		// not to be used in the match step.
		static double PureCTSF_TensorDistance(const Point* p1, const Point* p2); // for SWC correspondence list during estimation
		// for the match step
		static double EuclideanDistance(const Point *p1, const Point *p2, const double weight);
		static double CTSF_TensorDistance(const Point *p1, const Point *p2, const double weight);
		static double JDiff_TensorDistance(const Point *p1, const Point *p2, const double weight);
		static double LieDirectDistance(const Point *p1, const Point *p2, const double weight);

		// for computing the distances
		Eigen::Matrix3d* GetTensorMatrix() const;
		Eigen::Vector3d* GetTensorEigenValues() const;

		Eigen::Vector3d* GetTensorJValues() const;


    protected:

    private:
		Eigen::Vector3d position;
		Eigen::Vector3d color;
		Eigen::Vector3d normal;
		bool removed = false;

		Tensor* tensor = nullptr;


};

#endif // POINT_H