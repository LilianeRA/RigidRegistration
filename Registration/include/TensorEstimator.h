#ifndef TENSORESTIMATOR_H
#define TENSORESTIMATOR_H

#include "PointCloud.h"

class TensorEstimator
{
    public:
        TensorEstimator();
        virtual ~TensorEstimator();
	
        static void Estimate(const PointCloud *pointCloud, const bool regularization, const double limit_angle, 
            const double ellipsoid_angle, const double sigmaN, const double ctsf_percentage);
        static void SetLieTensors(const PointCloud* pointCloud);
    private:

        static void RadialStructuringElement(const PointCloud* pointCloud, const double sigmaN, const double ctsf_percentage);
        static void CoplanarStructuringElement(const PointCloud* pointCloud, const bool regularization, const double limit_angle, const double ellipsoid_angle, const double sigmaN);
};

#endif // TENSOR_H