#ifndef RIGIDREGISTRATION_H
#define RIGIDREGISTRATION_H


#include "MethodsData.h"

class RigidRegistration 
{
    public:

        RigidRegistration(const MethodsData *methodsData);
        virtual ~RigidRegistration();

        void Run();

    private:
		const MethodsData* data; // the first time this variable is seted, its done

        double currIterationWeight = 1e5;
        double maxIterationWeight = 1e5;
        double minIterationWeight = 1e-6;
        const double stepIterationWeight = 0.1;

        const double initialError = 1e10;
        double trimming = 1.0;
        int correspondences = 0; 

        std::vector<unsigned int> tgt2src_correspondence;
        std::vector<unsigned int> tgt2src_tensorCorrespondence;
        //std::vector<unsigned int> src2tgt_correspondence;

        std::function< double(const Point*, const Point*, const double, const bool) > distanceFunction;
        std::function< const Eigen::Affine3d(const PointCloud*, const PointCloud* , 
            const std::vector<unsigned int>&, const double ) > estimationFunction;

        std::function< void(const PointCloud*, const double) > preMatchFunction; // Only when using Lie Groups

        std::function< double(const PointCloud*, const PointCloud*, 
            const std::vector<unsigned int>&, const std::vector<unsigned int>& ,
            const Eigen::Affine3d&, const double) > RMS_Error;

        void Setup();
        void MatchPointClouds(const PointCloud* sourcemesh, const PointCloud* targetmesh);

        void SetTensorCorrespondenceList();

        static double RootMeanSquareOfTransformation(const PointCloud* sourcemesh, const PointCloud* targetmesh,
            const std::vector<unsigned int>& tgt2src_correspondence,
            const std::vector<unsigned int>& tgt2src_tensorCorrespondence,
            const Eigen::Affine3d &transformation, const double weight);

        static double RootMeanSquareSWC(const PointCloud* sourcemesh, const PointCloud* targetmesh,
            const std::vector<unsigned int>& tgt2src_correspondence,
            const std::vector<unsigned int>& tgt2src_tensorCorrespondence,
            const Eigen::Affine3d& transformation, const double weight);

        static double RootMeanSquare(const PointCloud* sourcemesh, const PointCloud* targetmesh,
            const std::vector<unsigned int>& tgt2src_correspondence);

};

#endif // RIGIDREGISTRATION_H
