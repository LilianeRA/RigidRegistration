#ifndef TENSOR_H
#define TENSOR_H

#include <Eigen/Dense>

class Tensor
{
    public:
        Tensor();
        virtual ~Tensor();
	    
        void Update(const Eigen::Matrix3d& tensorMatrix);
        void UpdateLieDirect(const Eigen::Vector3d& point_position, const double weight, bool verbose);
        void UpdateLieIndirect(const Eigen::Vector3d& point_position, const double weight);
        void UpdateLieGong(const Eigen::Vector3d& point_position, const double weight);
        void UpdateLieCalvo(const Eigen::Vector3d& point_position, const double weight);
        void UpdateLieLovric(const Eigen::Vector3d& point_position, const double weight);

        double GetPlanarCoefficient() const;
        const Eigen::Matrix3d& GetEigenVectors() const;

        const Eigen::Matrix3d& GetTensorMatrix() const;
        const Eigen::Vector3d& GetEigenValues() const; // used when computing the CTSF distance
        const Eigen::Vector3d& GetJValues() const; 

        const Eigen::Matrix4d& GetLieMatrix() const;
        const Eigen::Vector4d& GetLieEigenValues() const; // used when computing the Lie distance
    protected:

    private:
        Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
        Eigen::Vector3d eigenValues;
        Eigen::Matrix3d eigenVectors;

        double linearCoefficient = 0.0;
        double planarCoefficient = 0.0;
        double sphericalCoefficient = 0.0;
        double weight = 0.0;

        double vm = 0.0;
        double J1 = 0.0;
        double J2 = 0.0;
        double J3 = 0.0;

        Eigen::Matrix4d lieMatrix = Eigen::Matrix4d::Identity();
        Eigen::Vector4d lieEigenValues;
};

#endif // TENSOR_H