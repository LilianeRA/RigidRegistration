#include "RigidRegistration.h"
#include "TensorEstimator.h"
#include "DirHandler.h"
#include "Estimators.h"
#include "pch.h"
#include <iostream>
#include <iomanip>

void foo(const PointCloud*, const double)
{

}

RigidRegistration::RigidRegistration(MethodsData *methodsData)
{
    //ctor
	data = methodsData;
    Setup();
}

RigidRegistration::~RigidRegistration()
{
    //dtor
} 

MethodsData* RigidRegistration::GetMethodsData()
{
    return data;
}

void RigidRegistration::Reset()
{
    Setup();
}


const std::vector<Eigen::Affine3d>& RigidRegistration::Run()
{
    std::cout << "RigidRegistration::Run()\n";
    std::vector<Eigen::Affine3d> transformations;

    MethodsData::MODE mode;
    MethodsData::METHOD method;
    MethodsData::MATCH match;
    MethodsData::ESTIMATION estimation;
    data->getActiveMethod(mode, method, match, estimation);

    //const PointCloud* sourcemesh = data->getSourcePointCloud();
    //PointCloud* targetmesh = data->getTargetPointCloud()->Copy();
    PointCloud* sourcemesh = data->getSourcePointCloud()->Copy();
    const PointCloud* targetmesh = data->getTargetPointCloud(); // The target doesn't move

    const std::string test_directory = data->GetTestDirectory();
    //std::cout << "test_directory " << test_directory << "\n";
    
    unsigned int currentErrorIterations = 0;
    unsigned int currentIterations = 0;
    double currentError = initialError;
    double previousError = initialError;
    const int cout_precision = 6;
    std::cout << "---- Starting iterations ----\n"; // its a good warning because some matches are heavy to compute.
    while (currIterationWeight > 0.0 && currentError > 1e-12)
    {
        if (std::abs(currIterationWeight - minIterationWeight) < 0.0000077 || currIterationWeight < minIterationWeight)
        {
            if (match == MethodsData::MATCH::LIEDIR || match == MethodsData::MATCH::LIEIND || 
                match == MethodsData::MATCH::GONG || match == MethodsData::MATCH::CALVO || match == MethodsData::MATCH::LOVRIC )
            {
                // If you set the Lie Matrix to zero, it will get stuck while computing the log of the matrix (In LIEDIR).
                // So, don't continue it you need to set the weight to zero.
                break;
            }
            std::cout << std::scientific << std::setprecision(cout_precision) << "Setting weight to zero: cur. weight: ";
            std::cout << currIterationWeight << " <= " << minIterationWeight << " = min. weight\n";
            currIterationWeight = 0.0;
        }
        //std::cout << "MatchPointClouds\n";
        MatchPointClouds(sourcemesh, targetmesh);
        //const Eigen::Affine3d transformation = estimationFunction(sourcemesh, targetmesh, tgt2src_correspondence, currIterationWeight);
        //std::cout << "estimationFunction\n";
        const Eigen::Affine3d transformation = estimationFunction(sourcemesh, targetmesh, src2tgt_correspondence, currIterationWeight);
        //std::cout << std::scientific << std::setprecision(cout_precision) << "transformation\n" << transformation.matrix() << std::endl;
        ++currentErrorIterations;
        ++currentIterations;

        previousError = currentError;
        // compute error
        //std::cout << "RMS_Error\n";
        //currentError = RMS_Error(sourcemesh, targetmesh, tgt2src_correspondence, tgt2src_tensorCorrespondence, transformation, currIterationWeight);
        currentError = RMS_Error(sourcemesh, targetmesh, src2tgt_correspondence, src2tgt_tensorCorrespondence, transformation, currIterationWeight);

        std::cout << "It. " << currentIterations << ": ";
        // the method can be stuck improving the error in the 9th decimal. If this happens, it can do it up to 200 times.
        if (previousError - currentError > 1e-10 && currentErrorIterations < 200)  
        {
            std::cout << std::scientific << std::setprecision(cout_precision) << "Error improved from " << previousError << " to " << currentError << std::endl;
            // accept transformation
            //targetmesh->ApplyTransformation(transformation);
            //const std::string pointCloudPreffix{std::to_string(currentIterations)+"_"};
            //targetmesh->SaveCurrentPoints(DirHandler::JoinPaths(test_directory, pointCloudPreffix));

            sourcemesh->ApplyTransformation(transformation);
            const std::string pointCloudPreffix{ std::to_string(currentIterations) + "_" };
            sourcemesh->SaveCurrentPoints(DirHandler::JoinPaths(test_directory, pointCloudPreffix));

            transformations.push_back(transformation);
        }
        else
        {
            std::cout << std::scientific << std::setprecision(cout_precision) << "Error did not improved (" << previousError << " <= " << currentError <<"). " ;
            currentError = previousError;
            if (method == MethodsData::METHOD::ICP && match == MethodsData::MATCH::ICP && estimation == MethodsData::ESTIMATION::ICP)
            {
                std::cout << "Stoping.\n";
                currIterationWeight = 0.0;
            }
            else
            {
                std::cout << "Updating weight from "<< currIterationWeight << " to " ;

                currIterationWeight *= stepIterationWeight;
                std::cout << currIterationWeight << "\n";

                currentErrorIterations = 0;
            }
        }
        //int nada; std::cin >> nada;
    }

    // after the resgistration, compute the correspondent points
    distanceFunction = Point::EuclideanDistance; // For the Match Function to update the correspondence list, now based on euclidean distance
    preMatchFunction = foo;
    //MatchPointClouds(data->getTargetPointCloud(), targetmesh); 
    //currentError = RootMeanSquare(data->getTargetPointCloud(), targetmesh, tgt2src_correspondence);
    MatchPointClouds(sourcemesh, targetmesh);
    currentError = RootMeanSquare(sourcemesh, targetmesh, src2tgt_correspondence);

    unsigned int i = 0;
    unsigned int correspondences = 0;
    /*for (unsigned int t : tgt2src_correspondence)
    {
        if (i == t) 
        {
            correspondences++;
        }
        ++i;
    }*/
    for (unsigned int t : src2tgt_correspondence)
    {
        if (i == t) 
        {
            correspondences++;
        }
        ++i;
    }
    std::cout << "Final Euclidean error: " << currentError << " correspondences " << correspondences << std::endl;

    return transformations;
}

void RigidRegistration::Setup()
{
    std::cout << "RigidRegistration::Setup()\n";
    MethodsData::MODE mode;
    MethodsData::METHOD method;
    MethodsData::MATCH match;
    MethodsData::ESTIMATION estimation;
    data->getActiveMethod(mode, method, match, estimation);


    // original ICP, ICP-CTSF
    if (method == MethodsData::METHOD::ICP && estimation == MethodsData::ESTIMATION::ICP)
    {
        currIterationWeight = maxIterationWeight;
        minIterationWeight = 1e-6;

        if (match == MethodsData::MATCH::ICP)               // ICP 
        {
            distanceFunction = Point::EuclideanDistance;
            preMatchFunction = foo;
        }
        else if (match == MethodsData::MATCH::CTSF)               // ICP-CTSF
        {
            distanceFunction = Point::CTSF_TensorDistance;
            preMatchFunction = foo;
        }
        else if (match == MethodsData::MATCH::LIEDIR)             // ICP-LIEDIR
        {
            distanceFunction = Point::LieDirectDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieDirect;
        }
        else if (match == MethodsData::MATCH::LIEIND)             // ICP-LIEIND
        {
            distanceFunction = Point::LieIndirectDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieIndirect;
        }
        else if (match == MethodsData::MATCH::GONG)               // ICP-GONG
        {
            distanceFunction = Point::LieGongDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieGong;
        }
        else if (match == MethodsData::MATCH::CALVO)              // ICP-CALVO
        {
            distanceFunction = Point::LieCalvoDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieCalvo;
        }
        else if (match == MethodsData::MATCH::LOVRIC)             // ICP-LOVRIC
        {
            distanceFunction = Point::LieLovricDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieLovric;
        }
        else
        {
            std::cout<<"RigidRegistration::Setup(): Error: full method is not defined. Stopping\n";
            exit(-1);
        }

        estimationFunction = Estimators::ICP_Besl;

        RMS_Error = RootMeanSquareOfTransformation;
    }
    // SWC-ICP, SWC-CTSF, SWC-LIEDIR
    else if ( method == MethodsData::METHOD::ICP && estimation == MethodsData::ESTIMATION::SWC)
    {
        currIterationWeight = maxIterationWeight;
        minIterationWeight = 1e-6;

        if (match == MethodsData::MATCH::ICP)               // SWC-ICP
        {
            distanceFunction = Point::EuclideanDistance;
            preMatchFunction = foo;
        }
        else if(match == MethodsData::MATCH::CTSF)               // SWC-CTSF
        {
            distanceFunction = Point::CTSF_TensorDistance;
            preMatchFunction = foo;
        }
        else if(match == MethodsData::MATCH::LIEDIR)             // SWC-LIEDIR
        {
            distanceFunction = Point::LieDirectDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieDirect;
        }
        else if (match == MethodsData::MATCH::LIEIND)             // SWC-LIEIND
        {
            distanceFunction = Point::LieIndirectDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieIndirect;
        }
        else if (match == MethodsData::MATCH::GONG)               // SWC-GONG
        {
            distanceFunction = Point::LieGongDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieGong;
        }
        else if (match == MethodsData::MATCH::CALVO)              // SWC-CALVO
        {
            distanceFunction = Point::LieCalvoDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieCalvo;
        }
        else if (match == MethodsData::MATCH::LOVRIC)             // SWC-LOVRIC
        {
            distanceFunction = Point::LieLovricDistance;
            preMatchFunction = TensorEstimator::SetTensorsLieLovric;
        }
        else
        {
            std::cout << "RigidRegistration::Setup(): Error: full method is not defined. Stopping\n";
            exit(-1);
        }

        estimationFunction = Estimators::SWC_Akio;

        RMS_Error = RootMeanSquareSWC;

        // set correspondence list based on CTSF
        if (!data->getSourcePointCloud()->IsCTSF_DistanceListSet() || !data->getTargetPointCloud()->IsCTSF_DistanceListSet())
        {
            PRINT_ERROR("Error: Need to set the CTSF distance list before running the SWC method.");
            exit(-1);
        }
        SetTensorCorrespondenceList();
        //Estimators::SetTensorCorrespondenceList(data->getSourcePointCloud(), tgt2src_tensorCorrespondence);
        Estimators::SetTensorCorrespondenceList(data->getTargetPointCloud(), src2tgt_tensorCorrespondence);
    }

}

void RigidRegistration::SetTensorCorrespondenceList()
{
    // building the correspondence list
    /*tgt2src_tensorCorrespondence.clear();

    const PointCloud* sourcemesh = data->getSourcePointCloud();
    const PointCloud* targetmesh = data->getTargetPointCloud();
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();
    for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
    {
        const Point* tgt_pt = target_points.at(targetCounter);
        tgt2src_tensorCorrespondence.push_back(targetCounter);

        double distance = DBL_MAX;
        for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
        {
            const Point* src_pt = source_points.at(sourceCounter);
            double d = Point::PureCTSF_TensorDistance(src_pt, tgt_pt);
            if (d < distance)
            {
                distance = d;
                tgt2src_tensorCorrespondence.back() = sourceCounter;
            }
        }
    }*/
    // building the correspondence list
    src2tgt_tensorCorrespondence.clear();

    const PointCloud* sourcemesh = data->getSourcePointCloud();
    const PointCloud* targetmesh = data->getTargetPointCloud();
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();
    for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
    {
        const Point* src_pt = source_points.at(sourceCounter);
        src2tgt_tensorCorrespondence.push_back(sourceCounter);

        double distance = DBL_MAX;
        for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
        {
            const Point* tgt_pt = target_points.at(targetCounter);
            double d = Point::PureCTSF_TensorDistance(src_pt, tgt_pt);
            if (d < distance)
            {
                distance = d;
                src2tgt_tensorCorrespondence.back() = targetCounter;
            }
        }
    }
}

void RigidRegistration::MatchPointClouds(const PointCloud* sourcemesh, const PointCloud* targetmesh)
{
    // building the correspondence list
    /*tgt2src_correspondence.clear();

    //const PointCloud* sourcemesh = data->getSourcePointCloud();
    //const PointCloud* targetmesh = data->getTargetPointCloud();

    preMatchFunction(sourcemesh, currIterationWeight);
    preMatchFunction(targetmesh, currIterationWeight);

    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();
    for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
    {
        const Point* tgt_pt = target_points.at(targetCounter);
        if (!tgt_pt)
        {
            std::cout << "No tgt_pt! " << targetCounter << std::endl;
            exit(-1);
        }
        tgt2src_correspondence.push_back(targetCounter);

        double distance = DBL_MAX;
        for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
        {
            const Point* src_pt = source_points.at(sourceCounter);
            if (!src_pt)
            {
                std::cout << "No src_point! " << sourceCounter << std::endl;
                exit(-1);
            }
            double d = distanceFunction(src_pt, tgt_pt, currIterationWeight, false);
            if (d < distance)
            {
                distance = d;
                tgt2src_correspondence.back() = sourceCounter;
                /*if (sourceCounter == 27)
                {
                    distanceFunction(src_pt, tgt_pt, currIterationWeight, true);
                    std::cout << "tc " << targetCounter << " sc " << sourceCounter << " d " << distance << std::endl;
                    std::cout << "Continue: "; int nada; std::cin >> nada;
                }* /
            }
        }
    }*/
    
    /*for (int i = 0; i < tgt2src_correspondence.size(); i++)
    {
        std::cout << std::setprecision(20) << "i " << i << " " << tgt2src_correspondence.at(i);
        std::cout << " dist " << distanceFunction(source_points.at(tgt2src_correspondence.at(i)), target_points.at(i), currIterationWeight, true);
        std::cout << " pos " << source_points.at(tgt2src_correspondence.at(i))->GetPosition().transpose() << std::endl;
    }

    
    src2tgt_correspondence.clear();
    for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
    {
        const Point* src_pt = source_points.at(sourceCounter);
        src2tgt_correspondence.push_back(sourceCounter);

        double distance = DBL_MAX;
        for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
        {
            const Point* tgt_pt = target_points.at(targetCounter);
            double d = distanceFunction(src_pt, tgt_pt, currIterationWeight);
            if (d < distance)
            {
                distance = d;
                src2tgt_correspondence.back() = targetCounter;
            }
        }
    }
    std::for_each(tgt2src_correspondence.begin(), tgt2src_correspondence.end(),
        [](const int a) 
        {
            std::cout << a << " ";
        });*/



    // building the correspondence list
    src2tgt_correspondence.clear();

    preMatchFunction(sourcemesh, currIterationWeight);
    preMatchFunction(targetmesh, currIterationWeight);

    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();
    for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
    {
        const Point* src_pt = source_points.at(sourceCounter);
        if (!src_pt)
        {
            std::cout << "No src_point! " << sourceCounter << std::endl;
            exit(-1);
        }
        src2tgt_correspondence.push_back(sourceCounter);

        double distance = DBL_MAX;
        for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
        {
            const Point* tgt_pt = target_points.at(targetCounter);
            if (!tgt_pt)
            {
                std::cout << "No tgt_pt! " << targetCounter << std::endl;
                exit(-1);
            }
            double d = distanceFunction(src_pt, tgt_pt, currIterationWeight, false);
            if (d < distance)
            {
                distance = d;
                src2tgt_correspondence.back() = targetCounter;
            }
        }
    }
}

double RigidRegistration::RootMeanSquareOfTransformation(const PointCloud* sourcemesh, const PointCloud* targetmesh,
    const std::vector<unsigned int>& src2tgt_correspondence,
    const std::vector<unsigned int>& src2tgt_tensorCorrespondence,
    const Eigen::Affine3d& transformation, const double weight)
{
    /*const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    Eigen::Vector3d targetCentroid = Estimators::ComputeCenteroid(target_points);

    for (unsigned int index = 0; index < tgt2src_correspondence.size(); ++index)
    {
        Eigen::Vector3d targetPoint = target_points.at(index)->GetPosition();
        Eigen::Vector3d sourcePoint = source_points.at(tgt2src_correspondence.at(index))->GetPosition();
        targetPoint -= targetCentroid;
        targetPoint = transformation * targetPoint;
        targetPoint += targetCentroid;
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }
    error /= (double) tgt2src_correspondence.size();
    return std::sqrt(error);*/
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    Eigen::Vector3d sourceCentroid = Estimators::ComputeCenteroid(source_points);

    for (unsigned int index = 0; index < src2tgt_correspondence.size(); ++index)
    {
        Eigen::Vector3d sourcePoint = source_points.at(index)->GetPosition();
        Eigen::Vector3d targetPoint = target_points.at(src2tgt_correspondence.at(index))->GetPosition();
        sourcePoint -= sourceCentroid;
        sourcePoint = transformation * sourcePoint;
        sourcePoint += sourceCentroid;
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }
    error /= (double)src2tgt_correspondence.size();
    return std::sqrt(error);
}

double RigidRegistration::RootMeanSquareSWC(const PointCloud* sourcemesh, const PointCloud* targetmesh,
    const std::vector<unsigned int>& src2tgt_correspondence,
    const std::vector<unsigned int>& src2tgt_tensorCorrespondence,
    const Eigen::Affine3d& transformation, const double weight)
{
    /*const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    Eigen::Vector3d targetCentroid = Estimators::ComputeCenteroid(target_points);

    for (unsigned int index = 0; index < tgt2src_correspondence.size(); ++index)
    {
        Eigen::Vector3d targetPoint = target_points.at(index)->GetPosition();
        Eigen::Vector3d sourcePoint = source_points.at(tgt2src_correspondence.at(index))->GetPosition();
        if (std::abs(weight) > 0.0000077)
        {
            sourcePoint = (sourcePoint + weight * source_points.at(tgt2src_tensorCorrespondence.at(index))->GetPosition()) / (weight + 1);
        }
        targetPoint -= targetCentroid;
        targetPoint = transformation * targetPoint;
        targetPoint += targetCentroid;
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }

    error /= (double)tgt2src_correspondence.size();
    return std::sqrt(error);*/


    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    Eigen::Vector3d sourceCentroid = Estimators::ComputeCenteroid(source_points);

    for (unsigned int index = 0; index < src2tgt_correspondence.size(); ++index)
    {
        Eigen::Vector3d sourcePoint = source_points.at(index)->GetPosition();
        Eigen::Vector3d targetPoint = target_points.at(src2tgt_correspondence.at(index))->GetPosition();
        if (std::abs(weight) > 0.0000077)
        {
            targetPoint = (targetPoint + weight * target_points.at(src2tgt_tensorCorrespondence.at(index))->GetPosition()) / (weight + 1);
        }
        sourcePoint -= sourceCentroid;
        sourcePoint = transformation * sourcePoint;
        sourcePoint += sourceCentroid;
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }

    error /= (double)src2tgt_correspondence.size();
    return std::sqrt(error);
}

double RigidRegistration::RootMeanSquare(const PointCloud* sourcemesh, const PointCloud* targetmesh,
    const std::vector<unsigned int>& src2tgt_correspondence)
{
    /*const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    int c = 0;
    Eigen::Vector3d targetCentroid = Estimators::ComputeCenteroid(target_points);

    for (unsigned int index = 0; index < tgt2src_correspondence.size(); ++index)
    {
        Eigen::Vector3d targetPoint = target_points.at(index)->GetPosition();
        Eigen::Vector3d sourcePoint = source_points.at(tgt2src_correspondence.at(index))->GetPosition();
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }
    error /= (double)tgt2src_correspondence.size();
    return std::sqrt(error);*/

    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    int c = 0;
    Eigen::Vector3d targetCentroid = Estimators::ComputeCenteroid(target_points);

    for (unsigned int index = 0; index < src2tgt_correspondence.size(); ++index)
    {
        Eigen::Vector3d sourcePoint = source_points.at(index)->GetPosition();
        Eigen::Vector3d targetPoint = target_points.at(src2tgt_correspondence.at(index))->GetPosition();
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }
    error /= (double)src2tgt_correspondence.size();
    return std::sqrt(error);
}
