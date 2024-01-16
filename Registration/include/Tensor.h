#ifndef TENSOR_H
#define TENSOR_H

#include <Eigen/Dense>

class Tensor
{
    public:
        Tensor();
        virtual ~Tensor();
	    
        void Update(const Eigen::Matrix3d& tensorMatrix);
        void UpdateLieDirect(const Eigen::Vector3d& point_position);
        void UpdateLieIndirect(const Eigen::Vector3d& point_position);
        void UpdateLieGong(const Eigen::Vector3d& point_position);
        void UpdateLieCalvo(const Eigen::Vector3d& point_position);
        void UpdateLieLovric(const Eigen::Vector3d& point_position);

        double GetPlanarCoefficient() const;
        const Eigen::Matrix3d& GetEigenVectors() const;

        const Eigen::Matrix3d& GetTensorMatrix() const;
        const Eigen::Vector3d& GetEigenValues() const; // used when computing the CTSF distance
        const Eigen::Vector3d& GetJValues() const; 

        const Eigen::Matrix4d& GetLieMatrix() const;
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
};

#endif // TENSOR_H