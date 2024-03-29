#include "Tensor.h"
#include <iostream>
#include <iomanip>
#include <unsupported/Eigen/MatrixFunctions> // for matrix.log()

Tensor::Tensor()
{
    //ctor
}

Tensor::~Tensor()
{
    //dtor
}

Tensor* Tensor::Copy() const
{
    Tensor* copyTensor = new Tensor();

    copyTensor->matrix = this->matrix;
    copyTensor->eigenValues = this->eigenValues;
    copyTensor->eigenVectors = this->eigenVectors;

    copyTensor->linearCoefficient = this->linearCoefficient;
    copyTensor->planarCoefficient = this->planarCoefficient;
    copyTensor->sphericalCoefficient = this->sphericalCoefficient;

    copyTensor->vm = this->vm;
    copyTensor->J1 = this->J1;
    copyTensor->J2 = this->J2;
    copyTensor->J3 = this->J3;
    copyTensor->weight = this->weight;

    copyTensor->lieMatrix = this->lieMatrix;
    copyTensor->lieEigenValues = this->lieEigenValues;

    return copyTensor;
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

// This method os for embedding the multivariate Gaussians (aka CTSF) into a linear space
// Please read the "Local Log-Euclidean Multivariate Gaussian Descriptor and Its Application to Image Classification".
// Trust me, its worth it
// "The first method, called direct embedding Log-Euclidean(DE - LogE), maps $A^{+}(n + 1)$ via matrix logarithm to the linear space $A(n + 1)$."
void Tensor::UpdateLieDirect(const Eigen::Vector3d& point_position, const double weight, bool verbose)
{
    // A =
    //      L^{-T} \nu
    //      0      1
    // DE-LogE = ln(A)
    // \Sigma^{-1} = L \cdot L^T, where L is a lower triangular matrix
    
    const Eigen::Matrix3d ctsf_matrix_inverse = matrix.inverse();
    assert(std::abs( ((matrix * ctsf_matrix_inverse) - Eigen::Matrix3d::Identity()).norm()  ) < 0.000077 );

    Eigen::LLT<Eigen::Matrix3d> llt_of_A(ctsf_matrix_inverse);
    const Eigen::Matrix3d matrixL = llt_of_A.matrixL();
    if (verbose)
    {
        std::cout << std::setprecision(15) << "S\n" << matrix << std::endl;
        std::cout << "mean " << point_position.transpose() << std::endl;
        std::cout << "Sinv\n" << ctsf_matrix_inverse << std::endl;
        std::cout << "L\n" << matrixL << std::endl;
    }
    assert(std::abs(((matrixL * matrixL.transpose()) - ctsf_matrix_inverse).norm()) < 0.000077);

    // \omega * L^{-T}
    const Eigen::Matrix3d L_inverse_transpose = (weight * matrixL).inverse().transpose();
    if (verbose)
    {
        std::cout << "W*L\n" << (weight * matrixL) << std::endl;
        std::cout << "Linv\n" << (weight * matrixL).inverse() << std::endl;
        std::cout << "Linvtr\n" << L_inverse_transpose << std::endl;
    }

    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++)
        {
            lieMatrix(i, j) = L_inverse_transpose(i, j);
        }
        lieMatrix(i, 3) = point_position(i);
        lieMatrix(3, i) = 0.0;
    }
    lieMatrix(3, 3) = 1.0;
    if (verbose) std::cout << "A\n" << lieMatrix << std::endl;

    lieMatrix = lieMatrix.log();
    if (verbose) std::cout << "logA\n" << lieMatrix << std::endl;
    

    Eigen::SelfAdjointEigenSolver <Eigen::MatrixXd> solver(lieMatrix);
    // eigenvalues are sorted in increasing order
    lieEigenValues = solver.eigenvalues();
    if (verbose) std::cout << "lieEigenValues " << lieEigenValues.transpose() << std::endl;
    // we need it decreasing
    std::swap(lieEigenValues.x(), lieEigenValues.w());
    std::swap(lieEigenValues.y(), lieEigenValues.z());
    if (verbose) 
    {
        int nada;  
        std::cin >> nada;
    }
}

// "The second one, what we call indirect embedding Log-Euclidean(IE - LogE), first maps $A^{+}(n + 1)$ via the coset and
// polar decomposition into the space of symmetric positive definite(SPD) matrices, $Sym^{+}(n + 1)$, and then into the 
// linear space $Sym(n + 1)$ by the Log - Euclidean framework."
void Tensor::UpdateLieIndirect(const Eigen::Vector3d& point_position, const double weight)
{
    const Eigen::Matrix3d mean_mean_T = point_position * point_position.transpose();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            lieMatrix(i, j) = weight * matrix(i, j) + mean_mean_T(i, j);
        }
        lieMatrix(i, 3) = lieMatrix(3, i) = point_position(i);
    }
    lieMatrix(3, 3) = 1.0;

    lieMatrix = lieMatrix.sqrt();
    lieMatrix = lieMatrix.log();

    Eigen::SelfAdjointEigenSolver <Eigen::MatrixXd> solver(lieMatrix);
    // eigenvalues are sorted in increasing order
    lieEigenValues = solver.eigenvalues();
    // we need it decreasing
    std::swap(lieEigenValues.x(), lieEigenValues.w());
    std::swap(lieEigenValues.y(), lieEigenValues.z());
}

void Tensor::UpdateLieGong(const Eigen::Vector3d& point_position, const double weight)
{
    Eigen::LLT<Eigen::Matrix3d> llt_of_A(matrix);
    const Eigen::Matrix3d matrixL = llt_of_A.matrixL();
    const double sqrt_weight = std::sqrt(weight);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            lieMatrix(i, j) = sqrt_weight * matrixL(i, j);
        }
        lieMatrix(i, 3) = point_position(i);
        lieMatrix(3, i) = 0.0;
    }
    lieMatrix(3, 3) = 1.0;

    Eigen::SelfAdjointEigenSolver <Eigen::MatrixXd> solver(lieMatrix);
    // eigenvalues are sorted in increasing order
    lieEigenValues = solver.eigenvalues();
    // we need it decreasing
    std::swap(lieEigenValues.x(), lieEigenValues.w());
    std::swap(lieEigenValues.y(), lieEigenValues.z());
}

void Tensor::UpdateLieCalvo(const Eigen::Vector3d& point_position, const double weight)
{
    const Eigen::Matrix3d mean_mean_T = point_position * point_position.transpose();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            lieMatrix(i, j) = weight * matrix(i, j) + mean_mean_T(i, j);
        }
        lieMatrix(i, 3) = lieMatrix(3, i) = point_position(i);
    }
    lieMatrix(3, 3) = 1.0;

    Eigen::SelfAdjointEigenSolver <Eigen::MatrixXd> solver(lieMatrix);
    // eigenvalues are sorted in increasing order
    lieEigenValues = solver.eigenvalues();
    // we need it decreasing
    std::swap(lieEigenValues.x(), lieEigenValues.w());
    std::swap(lieEigenValues.y(), lieEigenValues.z());
}

void Tensor::UpdateLieLovric(const Eigen::Vector3d& point_position, const double weight)
{
    const Eigen::Matrix3d mean_mean_T = point_position * point_position.transpose();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            lieMatrix(i, j) = weight * matrix(i, j) + mean_mean_T(i, j);
        }
        lieMatrix(i, 3) = lieMatrix(3, i) = point_position(i);
    }
    lieMatrix(3, 3) = 1.0;
    // |\Sigma|^{-2/(n+1)}, n = 3
    // sqrt(|\Sigma|)
    lieMatrix = std::sqrt((weight * matrix).determinant()) * lieMatrix;

    Eigen::SelfAdjointEigenSolver <Eigen::MatrixXd> solver(lieMatrix);
    // eigenvalues are sorted in increasing order
    lieEigenValues = solver.eigenvalues();
    // we need it decreasing
    std::swap(lieEigenValues.x(), lieEigenValues.w());
    std::swap(lieEigenValues.y(), lieEigenValues.z());
}

double Tensor::GetPlanarCoefficient() const
{
    return planarCoefficient;
}

const Eigen::Matrix3d& Tensor::GetTensorMatrix() const
{
    return matrix;
}

const Eigen::Vector3d& Tensor::GetEigenValues() const
{
    return eigenValues;
}

const Eigen::Vector3d& Tensor::GetJValues() const
{
    return Eigen::Vector3d(J1, J2, J3);
}

const Eigen::Matrix4d& Tensor::GetLieMatrix() const
{
    return lieMatrix;
}

const Eigen::Vector4d& Tensor::GetLieEigenValues() const
{
    return lieEigenValues;
}

const Eigen::Matrix3d& Tensor::GetEigenVectors() const
{
    return eigenVectors;
}
