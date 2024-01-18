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

		Eigen::Vector3d GetNormal() const;
		void SetTensor(const Eigen::Matrix3d &tensorMatrix);
		// this is what differentiates the Lie approach (IE, DE, Gong, Calvo, Lovric). The result of this function is stored in "lieMatrix"
		bool SetTensorLieDirect(const double weight, bool verbose = false) const;
		bool SetTensorLieIndirect(const double weight) const;
		bool SetTensorLieGong(const double weight) const;
		bool SetTensorLieCalvo(const double weight) const;
		bool SetTensorLieLovric(const double weight) const;

		double GetTensorPlanarCoefficient() const;
		const Eigen::Matrix3d* GetTensorEigenVectors() const;

		// not to be used in the match step.
		static double PureCTSF_TensorDistance(const Point* p1, const Point* p2); // for SWC correspondence list during estimation
		// for the match step
		static double EuclideanDistance(const Point *p1, const Point *p2, const double weigh, const bool verbose = false);
		static double CTSF_TensorDistance(const Point *p1, const Point *p2, const double weight, const bool verbose = false);
		static double JDiff_TensorDistance(const Point *p1, const Point *p2, const double weight, const bool verbose = false);
		static double LieDirectDistance(const Point *p1, const Point *p2, const double weight, const bool verbose = false);
		static double LieIndirectDistance(const Point *p1, const Point *p2, const double weight, const bool verbose = false);
		static double LieGongDistance(const Point *p1, const Point *p2, const double weight, const bool verbose = false);
		static double LieCalvoDistance(const Point *p1, const Point *p2, const double weight, const bool verbose = false);
		static double LieLovricDistance(const Point *p1, const Point *p2, const double weight, const bool verbose = false);

		// for computing the distances
		const Eigen::Matrix3d* GetTensorMatrix() const;
		const Eigen::Vector3d* GetTensorEigenValues() const;
		const Eigen::Vector3d* GetTensorJValues() const;

		const Eigen::Matrix4d* GetLieMatrix() const; // return the value stored for the Lie approach (Direct, Indirect, Gong, Calvo, Lovric)
		const Eigen::Vector4d* GetLieEigenValues() const;


    protected:

    private:
		Eigen::Vector3d position;
		Eigen::Vector3d color;
		Eigen::Vector3d normal;
		bool removed = false;

		Tensor* tensor = nullptr;


};

#endif // POINT_H