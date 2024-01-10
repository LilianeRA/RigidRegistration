#include "Tensor.h"
#include <iostream>

Tensor::Tensor()
{
    //ctor
}

Tensor::~Tensor()
{
    //dtor
}

void Tensor::Update(const Eigen::Matrix3d& tensorMatrix)
{
    matrix = tensorMatrix.normalized();
    Eigen::SelfAdjointEigenSolver <Eigen::MatrixXd> solver(matrix);
    // eigenvalues are sorted in increasing order
    eigenValues = solver.eigenvalues();
    Eigen::Matrix3d eigenVecs = solver.eigenvectors();

    // we need it decreasing
    std::swap(eigenValues.x(), eigenValues.z());
    /*std::cout << "eigenValues " << eigenValues.transpose() << "\n";
    std::cout << "eigenVecs \n";
    std::cout << eigenVecs << "\n";*/
    
    eigenVecs.col(0) = eigenVecs.col(2);
    eigenVecs.col(2) = eigenVecs.col(0).cross(eigenVecs.col(1));
    eigenVectors = eigenVecs.transpose();

    /*Eigen::Vector3d aux = eigenVecs.col(0);
    eigenVecs.col(0) = eigenVecs.col(2);
    eigenVecs.col(2) = aux;
    eigenVectors = eigenVecs.transpose();*/

    /*std::cout << "eigenVectors after \n";
    std::cout << eigenVectors << "\n";*/

    double eigenValueSum = eigenValues.sum();

    if (std::abs(eigenValueSum) < eigenValueSum) 
    {
        linearCoefficient = planarCoefficient = sphericalCoefficient = 0.0;
    }
    else 
    {
        // cl =     (\lambda_1 - \lambda_2) / (\lambda_1 + \lambda_2 + \lambda_3)
        // cp = 2 * (\lambda_2 - \lambda_3) / (\lambda_1 + \lambda_2 + \lambda_3)
        // cs = 3 * \lambda_3               / (\lambda_1 + \lambda_2 + \lambda_3)
        linearCoefficient = (eigenValues(0) - eigenValues(1)) / eigenValueSum;
        planarCoefficient = 2.0 * (eigenValues(1) - eigenValues(2)) / eigenValueSum;
        sphericalCoefficient = 3.0 * eigenValues(2) / eigenValueSum;
    }

    //computing the tensor weight
    eigenValues = eigenValues.normalized();

    vm  = (eigenValues(0) - eigenValues(1)) * (eigenValues(0) - eigenValues(1));
    vm += (eigenValues(1) - eigenValues(2)) * (eigenValues(1) - eigenValues(2));
    vm += (eigenValues(0) - eigenValues(2)) * (eigenValues(0) - eigenValues(2));
    vm /= 2.0;

    J1 = eigenValues(0) + eigenValues(1) + eigenValues(2);
    J2 = eigenValues(0) * eigenValues(1) + eigenValues(0) * eigenValues(2) + eigenValues(1) * eigenValues(2);
    J3 = eigenValues(0) * eigenValues(1) * eigenValues(2);
   
    weight = planarCoefficient;
    //setGlyphParameters();
}

double Tensor::GetPlanarCoefficient()
{
    return planarCoefficient;
}

const Eigen::Matrix3d& Tensor::GetTensorMatrix()
{
    return matrix;
}

const Eigen::Vector3d& Tensor::GetEigenValues()
{
    return eigenValues;
}

const Eigen::Vector3d& Tensor::GetJValues()
{
    return Eigen::Vector3d(J1, J2, J3);
}

const Eigen::Matrix3d& Tensor::GetEigenVectors()
{
    return eigenVectors;
}
