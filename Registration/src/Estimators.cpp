#include "Estimators.h"
#include <iostream>
#include <iomanip>


std::vector<Point*> Estimators::CTSFcorrespondentPoints = std::vector<Point*>(0,nullptr);
Eigen::Vector3d Estimators::CTSF_CorrespCentroid = Eigen::Vector3d(-INT_MAX, -INT_MAX, -INT_MAX);

Estimators::Estimators()
{
    //ctor
}

Estimators::~Estimators()
{
    //dtor
}

const Eigen::Affine3d Estimators::ICP_Besl(const PointCloud* sourcemesh, const PointCloud* targetmesh, 
    const std::vector<unsigned int>& src2tgt_correspondence, const double weight)
{
    //std::cout << "ICP_Besl\n";
    // TODO: trimming

    // list of points from point cloud
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    std::vector<Point*> correspondentPoints;
    //unsigned int i = 0;
    /*for (const unsigned int src_index : tgt2src_correspondence)
    {
        correspondentPoints.push_back(source_points.at(src_index));
        if (i <= 10)
        {
            std::cout << std::setprecision(15) << i << " " << src_index << std::endl;
            std::cout << target_points.at(i)->GetPosition().transpose() << std::endl;
            std::cout << source_points.at(src_index)->GetPosition().transpose() << std::endl;
            std::cout << correspondentPoints.back()->GetPosition().transpose() << std::endl;

            ++i;
        }
    }*/
    for (const unsigned int tgt_index : src2tgt_correspondence)
    {
        correspondentPoints.push_back(target_points.at(tgt_index));
    }

    ////////Eigen::Vector3d targetCentroid = ComputeCenteroid(target_points);
    Eigen::Vector3d sourceCentroid = ComputeCenteroid(source_points);
    Eigen::Vector3d correspCentroid = ComputeCenteroid(correspondentPoints, true);
    //std::cout << std::setprecision(15) << "dataCenterOfMass " << targetCentroid.transpose() << "\n";
    //std::cout << "YCenterOfMass    " << correspCentroid.transpose() << "\n";
    //////const Eigen::Matrix3d covariance = ComputeCovariance(target_points, correspondentPoints, targetCentroid, correspCentroid);
    const Eigen::Matrix3d covariance = ComputeCovariance(source_points, correspondentPoints, sourceCentroid, correspCentroid);
    //std::cout << "covariance\n" << covariance << "\n";
    const Eigen::Matrix4d quaternionMatrix = ComputeQuaternionFromCovariance(covariance);
    //std::cout << "quaternionMat\n" << quaternionMatrix << "\n";

    // eigenvalues are sorted in increasing order
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quaternionMatrix);
    Eigen::Matrix4d eigenVectors = solver.eigenvectors();
    Eigen::Vector4d greatestEigenVec = eigenVectors.col(3); // the eigen vec of greatest eigenvalue
    //std::cout << "eigenVectors\n" << eigenVectors << "\n";
    //std::cout << "greatestEigenVec " << greatestEigenVec.transpose() << "\n";
    Eigen::Quaterniond rotationQuaternion{ greatestEigenVec.x(), greatestEigenVec.y(), greatestEigenVec.z(), greatestEigenVec.w() };
    //std::cout << "rotationQuaternion\n" << rotationQuaternion.matrix() << "\n";

    ////////Eigen::Affine3d transformation = Eigen::Affine3d(Eigen::Translation3d(correspCentroid - rotationQuaternion * targetCentroid) * rotationQuaternion);
    Eigen::Affine3d transformation = Eigen::Affine3d(Eigen::Translation3d(correspCentroid - rotationQuaternion * sourceCentroid) * rotationQuaternion);
    //std::cout << "transformation\n" << transformation.matrix() << "\n";

    return transformation;
}

const Eigen::Affine3d Estimators::SWC_Akio(const PointCloud* sourcemesh, const PointCloud* targetmesh,
    const std::vector<unsigned int>& src2tgt_correspondence, const double weight)
{
    //std::cout << "SWC_Akio\n";
    std::cout << std::setprecision(15);
    // list of points from point cloud
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    std::vector<Point*> correspondentPoints;
    //unsigned int i = 0;
    /*for (const unsigned int src_index : tgt2src_correspondence)
    {
        correspondentPoints.push_back(source_points.at(src_index));
        if (i <= 5)
        {
            std::cout << std::setprecision(15) << i << " " << src_index << std::endl;
            std::cout << target_points.at(i)->GetPosition().transpose() << std::endl;
            std::cout << source_points.at(src_index)->GetPosition().transpose() << std::endl;
            std::cout << correspondentPoints.back()->GetPosition().transpose() << std::endl;

            ++i;
        }
    }*/
    for (const unsigned int tgt_index : src2tgt_correspondence)
    {
        correspondentPoints.push_back(target_points.at(tgt_index));
    }

    ////////Eigen::Vector3d targetCentroid = ComputeCenteroid(target_points);
    Eigen::Vector3d sourceCentroid = ComputeCenteroid(source_points);
    Eigen::Vector3d correspCentroid = ComputeCenteroid(correspondentPoints);
    //std::cout << "dataCenterOfMass " << targetCentroid.transpose() << "\n";
    //std::cout << "YCenterOfMass    " << correspCentroid.transpose() << "\n";
    ///////////////Eigen::Matrix3d covariance = ComputeCovariance(target_points, correspondentPoints, targetCentroid, correspCentroid);
    Eigen::Matrix3d covariance = ComputeCovariance(source_points, correspondentPoints, sourceCentroid, correspCentroid);
    //std::cout << "covariance\n" << covariance << "\n";

    if (std::abs(weight) > 0.0000077) 
    {
        //std::cout << "YCenterOfMass2 " << CTSF_CorrespCentroid.transpose() << std::endl;
        // Compute the CTSF-based covariance matrix between both point clouds
        ////////////////const Eigen::Matrix3d cross_covariance = ComputeCovariance(target_points, CTSFcorrespondentPoints, targetCentroid, CTSF_CorrespCentroid);
        const Eigen::Matrix3d cross_covariance = ComputeCovariance(source_points, CTSFcorrespondentPoints, sourceCentroid, CTSF_CorrespCentroid);
        //std::cout << "covariance2" << std::endl;
        //std::cout << std::setprecision(15) << cross_covariance.matrix() << std::endl;

        /*if (wSch == W_EXPONENTIAL) {
            std::cout << "W_EXPONENTIAL" << std::endl;
            covariance = (covariance + this->W * covariance2);
        }
        else if (wSch == W_INTERPOLATION) {
            covariance = (1 - this->W) * covariance + (this->W * covariance2);
        }*/
        covariance = (covariance + weight * cross_covariance);
        //std::cout << "new covariance" << std::endl;
        //std::cout << std::setprecision(15) << cross_covariance.matrix() << std::endl;
    }

    const Eigen::Matrix4d quaternionMatrix = ComputeQuaternionFromCovariance(covariance);
    //std::cout << "quaternionMat\n" << quaternionMatrix << "\n";

    // eigenvalues are sorted in increasing order
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quaternionMatrix);
    Eigen::Matrix4d eigenVectors = solver.eigenvectors();
    Eigen::Vector4d greatestEigenVec = eigenVectors.col(3); // the eigen vec of greatest eigenvalue
    //std::cout << "eigenVectors\n" << eigenVectors << "\n";
    //std::cout << "greatestEigenVec " << greatestEigenVec.transpose() << "\n";
    Eigen::Quaterniond rotationQuaternion{ greatestEigenVec.x(), greatestEigenVec.y(), greatestEigenVec.z(), greatestEigenVec.w() };
    //std::cout << "rotationQuaternion\n" << rotationQuaternion.matrix() << "\n";


    /////////////Eigen::Affine3d transformation = Eigen::Affine3d(Eigen::Translation3d(correspCentroid - rotationQuaternion * targetCentroid) * rotationQuaternion);
    Eigen::Affine3d transformation = Eigen::Affine3d(Eigen::Translation3d(correspCentroid - rotationQuaternion * sourceCentroid) * rotationQuaternion);
    //std::cout << "transformation\n" << transformation.matrix() << "\n";

    //std::cout << "Enter number: "; int nada; std::cin >> nada;
    return transformation;
}

/*void Estimators::SetTensorCorrespondenceList(const PointCloud* sourcemesh, const std::vector<unsigned int>& tgt2src_tensorCorrespondence)
{
    CTSFcorrespondentPoints.clear();
    const auto& source_points = sourcemesh->GetPoints();
    for (const unsigned int src_index : tgt2src_tensorCorrespondence)
    {
        CTSFcorrespondentPoints.push_back(source_points.at(src_index));
    }
    CTSF_CorrespCentroid = ComputeCenteroid(CTSFcorrespondentPoints);
}*/
void Estimators::SetTensorCorrespondenceList(const PointCloud* targetmesh, const std::vector<unsigned int>& src2tgt_tensorCorrespondence)
{
    CTSFcorrespondentPoints.clear();
    const auto& target_points = targetmesh->GetPoints();
    for (const unsigned int tgt_index : src2tgt_tensorCorrespondence)
    {
        CTSFcorrespondentPoints.push_back(target_points.at(tgt_index));
    }
    CTSF_CorrespCentroid = ComputeCenteroid(CTSFcorrespondentPoints);
}



const Eigen::Vector3d Estimators::ComputeCenteroid(const std::vector<Point*> &points, bool verbose)
{
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);

    //int i = 0;
    for (const auto* p : points)
    {
        centroid += p->GetPosition();
        //if (verbose && i == 27) std::cout <<std::setprecision(20)<<i<<": "<< p->GetPosition().transpose()<<", "<< centroid.transpose() << std::endl;
        //i++;
    }
    ///std::cout << "centroid sum " << centroid.transpose() << std::endl;
    centroid /= (double)points.size();
    //std::cout << "centroid " << centroid.transpose() << std::endl;

    return centroid;
}

const Eigen::Matrix3d Estimators::ComputeCovariance(const std::vector<Point*>& source_points, const std::vector<Point*>& correspondentPoints,
    const Eigen::Vector3d& sourceCentroid, const Eigen::Vector3d& correspCentroid)
{
    Eigen::Matrix3d covariance;
    covariance.setZero(3, 3);

    Eigen::Matrix3d meanDifference = sourceCentroid * correspCentroid.transpose();

    for (unsigned int i = 0; i < source_points.size(); i++)
    {
        covariance += (source_points.at(i)->GetPosition() * correspondentPoints.at(i)->GetPosition().transpose()) - meanDifference;
    }
    covariance = covariance / (double)source_points.size();

    return covariance;
}

const Eigen::Matrix4d Estimators::ComputeQuaternionFromCovariance(const Eigen::Matrix3d& covariance)
{
    double covarianceTrace = covariance(0) + covariance(4) + covariance(8);
    Eigen::Matrix3d matrixA = covariance - covariance.transpose();
    Eigen::Matrix3d matrixT = covariance + covariance.transpose();

    // Subtracts the trace from the main diagonal
    matrixT(0) -= covarianceTrace;
    matrixT(4) -= covarianceTrace;
    matrixT(8) -= covarianceTrace;

    // Eigen::Vector3d vectorD{ matrixA[5], matrixA[6], matrixA[1] };
    Eigen::Matrix4d quaternionMatrix;
    quaternionMatrix << covarianceTrace, matrixA(1, 2), matrixA(2, 0), matrixA(0, 1),
        matrixA(1, 2), matrixT(0, 0), matrixT(0, 1), matrixT(0, 2),
        matrixA(2, 0), matrixT(1, 0), matrixT(1, 1), matrixT(1, 2),
        matrixA(0, 1), matrixT(2, 0), matrixT(2, 1), matrixT(2, 2);
    /*std::cout << "matrixA\n" << matrixA << std::endl;
    std::cout << "vectorD\n" << matrixA(1, 2) << " " << matrixA(2, 0) << " " << matrixA(0, 1) << std::endl;
    std::cout << "covarianceTrace " << covarianceTrace << std::endl;
    std::cout << "matrixT\n" << matrixT << std::endl;*/
    return quaternionMatrix;
}
