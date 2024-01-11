#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "PointCloud.h"

class Estimators 
{
    public:

        Estimators();
        virtual ~Estimators();

        static const Eigen::Vector3d ComputeCenteroid(const std::vector<Point*> &points);

        static const Eigen::Affine3d ICP_Besl(const PointCloud* sourcemesh, const PointCloud* targetmesh, const std::vector<unsigned int> &tgt2src_correspondence);
    private:

        static const Eigen::Matrix3d ComputeCovariance(const std::vector<Point*>& source_points, const std::vector<Point*>& correspondentPoints, 
            const Eigen::Vector3d &sourceCentroid, const Eigen::Vector3d &correspCentroid);
        static const Eigen::Matrix4d ComputeQuaternionFromCovariance(const Eigen::Matrix3d &covariance);
};

#endif // ESTIMATORS_H
