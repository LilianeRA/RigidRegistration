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

        int iterationCounter = 0;

        const double initialError = 1e10;
        double trimming = 1.0;
        int correspondences = 0; 

        std::vector<unsigned int> tgt2src_correspondence;
        std::vector<unsigned int> tgt2src_tensorCorrespondence;
        //std::vector<unsigned int> src2tgt_correspondence;

        std::function< double(const Point*, const Point*, const double) > distanceFunction;
        std::function< const Eigen::Affine3d(const PointCloud* sourcemesh, 
            const PointCloud* targetmesh, 
            const std::vector<unsigned int>& tgt2src_correspondence,
            const double weight) > estimationFunction;

        void Setup();
        void MatchPointClouds();

        void SetTensorCorrespondenceList();

        static double RootMeanSquareOfTransformation(const PointCloud* sourcemesh, const PointCloud* targetmesh,
            const std::vector<unsigned int>& tgt2src_correspondence, const Eigen::Affine3d &transformation);

        static double RootMeanSquare(const PointCloud* sourcemesh, const PointCloud* targetmesh,
            const std::vector<unsigned int>& tgt2src_correspondence);
};

#endif // RIGIDREGISTRATION_H
