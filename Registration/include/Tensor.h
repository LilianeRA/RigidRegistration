#ifndef TENSOR_H
#define TENSOR_H

#include <Eigen/Dense>

class Tensor
{
    public:
        Tensor();
        virtual ~Tensor();
	    
        void Update(const Eigen::Matrix3d& tensorMatrix);

        double GetPlanarCoefficient();
        const Eigen::Matrix3d& GetEigenVectors();

        //temporary
        const Eigen::Matrix3d& GetTensorMatrix();
        const Eigen::Vector3d& GetEigenValues();

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
};

#endif // TENSOR_H