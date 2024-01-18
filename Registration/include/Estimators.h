#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "PointCloud.h"

class Estimators 
{
    public:

        Estimators();
        virtual ~Estimators();

        static const Eigen::Vector3d ComputeCenteroid(const std::vector<Point*> &points, bool verbose = false);

        static const Eigen::Affine3d ICP_Besl(const PointCloud* sourcemesh, const PointCloud* targetmesh, 
            const std::vector<unsigned int> &tgt2src_correspondence, const double weight); // weight is ignored
        static const Eigen::Affine3d SWC_Akio(const PointCloud* sourcemesh, const PointCloud* targetmesh, 
            const std::vector<unsigned int> &tgt2src_correspondence, const double weight);
    
        static void SetTensorCorrespondenceList(const PointCloud* sourcemesh, const std::vector<unsigned int> & tgt2src_tensorCorrespondence); // for SWC
    private:
        // for SWC
        static std::vector<Point*> CTSFcorrespondentPoints;
        static Eigen::Vector3d CTSF_CorrespCentroid; 

        static const Eigen::Matrix3d ComputeCovariance(const std::vector<Point*>& source_points, const std::vector<Point*>& correspondentPoints, 
            const Eigen::Vector3d &sourceCentroid, const Eigen::Vector3d &correspCentroid);
        static const Eigen::Matrix4d ComputeQuaternionFromCovariance(const Eigen::Matrix3d &covariance);
};

#endif // ESTIMATORS_H
