#include "TensorEstimator.h"
#include "RandomGaussian.h"
#include "pch.h"
#include <numbers>
#include <fstream>

#define MAX_IT 100
#define K_INFLUENCE 0.01

//const std::string filepath{"E:/RigidRegistration/Mandar3/result/ICP/ICP_CTSF_05_1/regCoplanarSE.txt"};

TensorEstimator::TensorEstimator()
{
    //ctor
}

TensorEstimator::~TensorEstimator()
{
    //dtor
}
//const std::string radialDataFile{ "E:/RigidRegistration/Mandar3/result/ICP/ICP_CTSF_05_1/regRadialDataSE.txt" };
//const std::string coplanarDataFile{ "E:/RigidRegistration/Mandar3/result/ICP/ICP_CTSF_05_1/regCoplanarDataSE.txt" };

void TensorEstimator::Estimate(const PointCloud* pointCloud, const bool regularization, const double limit_angle, 
    const double ellipsoid_angle, const double sigmaN, const double ctsf_percentage)
{
    RadialStructuringElement(pointCloud, sigmaN, ctsf_percentage);

    /*std::ofstream file;
    file.open(radialDataFile.c_str());
    const auto& p = pointCloud->GetPoints();
    for (int x = 0; x < pointCloud->GetTotalPoints(); x++) {
        file << std::fixed << std::setprecision(15) << "n " << p.at(x)->GetNormal().transpose();
        file << "\n";

        file << *p.at(x)->GetTensorMatrix();
        file << "\n\n";
    }
    file.close();*/


    CoplanarStructuringElement(pointCloud, regularization, limit_angle, ellipsoid_angle, sigmaN);

    /*std::ofstream file2;
    file2.open(coplanarDataFile.c_str());
    const auto& p2 = pointCloud->GetPoints();
    for (int x = 0; x < pointCloud->GetTotalPoints(); x++) {
        file2 << std::fixed << std::setprecision(15) << "n " << p2.at(x)->GetNormal().transpose();
        file2 << "\n";

        file2 << *p2.at(x)->GetTensorMatrix();
        file2 << "\n\n";
    }
    file2.close();*/

    double cp = 0, prevcp = -1;
    int it = 0;
    // As long as the cp is raising the coplanarSE is applyed
    while (cp > prevcp && it < MAX_IT)
    {
        /*std::ofstream file;
        file.open(filepath.c_str(), std::ofstream::out | std::ofstream::app);
        file << std::fixed << std::setprecision(15) << "it: " << it << " cp " << cp << " prevcp " << prevcp << "\n";
        file.close();*/
        CoplanarStructuringElement(pointCloud, regularization, limit_angle, ellipsoid_angle, sigmaN);


        prevcp = cp;
        cp = 0;
        const auto& points = pointCloud->GetPoints();
        for (const Point *p : points) 
        {
            cp += p->GetTensorPlanarCoefficient();
        }
        cp /= points.size();

        /*file.open(filepath.c_str(), std::ofstream::out | std::ofstream::app);
        //std::cout << std::fixed << std::setprecision(15) << "it: " << it << " cp " << cp << "\n";
        for (int pointCounter = 0; pointCounter < points.size(); ++pointCounter)
        {
            //std::cout << "n " << points.at(pointCounter)->GetNormal().transpose() << "\n";
            //std::cout << *points.at(pointCounter)->GetTensorMatrix() << "\n\n";
            file << "x " << pointCounter<< " normal " << points.at(pointCounter)->GetNormal().transpose() << "\n";
            file << *points.at(pointCounter)->GetTensorMatrix() << "\n";
        }
        file.close();*/
        ++it;
    }
}

void TensorEstimator::SetLieTensors(const PointCloud* pointCloud)
{
    const auto& points = pointCloud->GetPoints();
    for(Point *point : points)
    {
        if (!point->SetLieTensor())
        {
            PRINT_ERROR("Error: no tensor for the point. Check if the CTSF tensor was estimated before.");
            exit(-1);
        }
    }
}

void TensorEstimator::RadialStructuringElement(const PointCloud* pointCloud, const double sigmaN, const double ctsf_percentage)
{
    RandomGaussian* random_gauss = new RandomGaussian();
    random_gauss->m_SetSeed(1, 1);

    Eigen::Matrix3d tensor_sum;

    /// Phase 1
    const auto& points = pointCloud->GetPoints();
    const int lastIndex = (ctsf_percentage == 100.0 ? -1 : std::floor(points.size() * ctsf_percentage)-2);
    //std::cout << "lastIndex " << lastIndex << "\n";
    //std::cout << "points.size() * ctsf_percentage " << points.size() * ctsf_percentage -1 << "\n";
    /*for (int pointCounter = 0; pointCounter < points.size(); ++pointCounter)
    {
        Eigen::Vector3d posX = points.at(pointCounter)->GetPosition();
        std::cout << posX.x() << " " << posX.y() << " " << posX.z() << "\n";
    }*/
    // for (int x = 0; x < pointCloud->nPoints; x++)
    for (int pointCounter = 0; pointCounter < points.size(); ++pointCounter)
    {
        //std::cout << "pointCounter " << pointCounter << "\n";
        
        // Initial tensor is a ball
        tensor_sum = Eigen::Matrix3d::Zero(3, 3);

        // Gets the farthest point in the neighborhood to set its influence
        //pq = pointCloud->list[pointCloud->invertedList(x, pointCloud->k - 1)]->pos - pointCloud->list[x]->pos;
        Eigen::Vector3d currentPoint = points.at(pointCounter)->GetPosition();
        Eigen::Vector3d fathestPoint = pointCloud->GetPointFromDistanceList(pointCounter, lastIndex)->GetPosition();
        Eigen::Vector3d pointToFathestPoint = fathestPoint - currentPoint;
        // \sigma_p = \sqrt{ || pq_f ||^2 / ln(0.01)}
        double max_ed2 = pointToFathestPoint.squaredNorm(); // the sum of the squared components
        // Sigma is set so the farthest point has 1% of influence
        double pointStandardDeviation = std::sqrt(-max_ed2 / std::log(K_INFLUENCE));
        
        
        //std::cout << "x " << pointCounter << ", " << pointCloud->GetIndexFromDistanceList(pointCounter, -1) << "\n";
        /*std::cout << "x " << pointCounter << ", " << lastIndex << "\n";
        std::cout << "pX  " << currentPoint.x() << " " << currentPoint.y() << " " << currentPoint.z() << "\n";
        std::cout << "pXK " << fathestPoint.x() << " " << fathestPoint.y() << " " << fathestPoint.z() << "\n";
        std::cout << "pq  " << pointToFathestPoint.x() << " " << pointToFathestPoint.y() << " " << pointToFathestPoint.z() << ", " << pointToFathestPoint.norm() << "\n";
        std::cout << "max_ed2  " << max_ed2 << "\n";
        std::cout << "log(mKInfluence) " << std::log(K_INFLUENCE) << " " << (-max_ed2 / std::log(K_INFLUENCE)) << "\n";
        std::cout << std::setprecision(20) << "pointCloud->sigma " << pointStandardDeviation << "\n";*/
        
        
        // Run the neighborhood
        // for (int i = 0; i < pointCloud->k; i++)
        // Note: the distance list does not include the vertex itself. Its size is points.size()-1 
        for (int neighborCounter = 0; neighborCounter < points.size()-1; ++neighborCounter) 
        {
            //pqAux = pointCloud->list[pointCloud->invertedList(x,i)]->pos - pointCloud->list[x]->pos;
            Eigen::Vector3d neighborPoint = pointCloud->GetPointFromDistanceList(pointCounter, neighborCounter)->GetPosition();
            Eigen::Vector3d pointToNeighborPoint = neighborPoint - currentPoint;
            //std::cout << "pqAux " << pointToNeighborPoint(0) << " " << pointToNeighborPoint(1) << " " << pointToNeighborPoint(2) << "\n";

            // Displace the point, so additive noise could be softened
            // pqAux = pqAux * rd * sigmaN;
            double rd = random_gauss->m_Random();
            pointToNeighborPoint = pointToNeighborPoint * rd * sigmaN;
            pointToFathestPoint = (neighborPoint + pointToNeighborPoint) - currentPoint;

            double dist2 = pointToFathestPoint.squaredNorm();
            
            /*std::cout << "pXI " << neighborPoint(0) << " " << neighborPoint(1) << " " << neighborPoint(2) << "\n";
            std::cout << "rd " << rd << "\n";
            std::cout << "pqAux " << pointToNeighborPoint(0) << " " << pointToNeighborPoint(1) << " " << pointToNeighborPoint(2) << "\n";
            std::cout << "pq    " << pointToFathestPoint(0) << " " << pointToFathestPoint(1) << " " << pointToFathestPoint(2) << "\n";
            std::cout << "dist2 " << dist2 << "\n";*/
            
            if (std::abs(dist2) > 0.0000077) 
            {
                // T_p = \Sum{e^{-||pq||^2 / \sigma^2_p} \cdot pq \cdot pq^T}
                // DIFF *****************************************************************
                //gaussian no original era float: gaussian 0.99983131885528564453
                //aqui é gaussian 0.99983134769956938381 
                double gaussian = std::exp(-dist2 / (pointStandardDeviation * pointStandardDeviation));
                Eigen::Matrix3d t = pointToFathestPoint * pointToFathestPoint.transpose();
                for (int j = 0; j < 9; j++)
                    tensor_sum(j) += gaussian * t(j);

                /*std::cout << "t        " << t(0) << " " << t(1) << " " << t(2) << "\n";
                std::cout << std::setprecision(20)<< "gaussian " << gaussian << "\n";
                std::cout << "tensorsum" << "\n";
                std::cout << tensor_sum << "\n";*/
            }
        }
        
        
        double l2 = 0.0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                l2 += tensor_sum(i, j) * tensor_sum(i, j);
        l2 = std::sqrt(l2);

        if (std::abs(l2) < 0.0000077)
        {
            tensor_sum.setIdentity(3, 3);
        }
        points.at(pointCounter)->SetTensor(tensor_sum);
        
        /*std::cout << "l2    " << l2 << "\n";
        std::cout << "updateTensor radial" << "\n";
        std::cout << *points.at(pointCounter)->GetTensorMatrix() << "\n\n";
        std::cout << *points.at(pointCounter)->GetTensorEigenValues() << "\n\n";
        std::cout << *points.at(pointCounter)->GetTensorEigenVectors() << "\n\n";
        std::cout << points.at(pointCounter)->GetNormal() << "\n\n";*/
    }
}

void TensorEstimator::CoplanarStructuringElement(const PointCloud* pointCloud, const bool regularization, const double limit_angle, const double ellipsoid_angle, const double sigmaN)
{
    //std::ofstream file;
    //file.open(filepath.c_str(), std::ofstream::out | std::ofstream::app);

    RandomGaussian* random_gauss = new RandomGaussian();
    random_gauss->m_SetSeed(1, 1);

    Eigen::MatrixXd tensor_sum = Eigen::MatrixXd::Zero(9, pointCloud->GetTotalPoints());

    /// Phase 2
   const auto& points = pointCloud->GetPoints();
    // for (int x = 0; x < pointCloud->nPoints; x++)
    for (int pointCounter = 0; pointCounter < points.size(); ++pointCounter)
    {
        //Mount a rotation matrix with the eigenvectors.
        Eigen::Matrix3d* eigenVectors = points.at(pointCounter)->GetTensorEigenVectors();
        if (!eigenVectors)
        {
            PRINT_ERROR("No tensor, no eigen vector");
            exit(-1);
        }
        //std::cout << std::fixed << std::setprecision(20) ;
        //std::cout << std::fixed << std::setprecision(20) << "x " << pointCounter << " Eigenvectors\n";
        //std::cout << *eigenVectors << "\n";

        eigenVectors->col(0).real().normalized();
        eigenVectors->col(1).real().normalized();
        eigenVectors->col(2).real().normalized();

        // Gets the farthest point in the neighborhood to set its influence
        Eigen::Vector3d currentPoint = points.at(pointCounter)->GetPosition();
        Eigen::Vector3d fathestPoint = pointCloud->GetFarthestPoint(pointCounter)->GetPosition();
        Eigen::Vector3d pointToFathestPoint = fathestPoint - currentPoint;

        // \sigma_p = \sqrt{ || pq_f ||^2 / ln(0.01)}
        double max_ed2 = pointToFathestPoint.squaredNorm(); // the sum of the squared components
        double pointStandardDeviation = std::sqrt(-max_ed2 / std::log(K_INFLUENCE));

        /*
        std::cout << "v\n";
        std::cout << *eigenVectors << "\n";
        std::cout << "pointCloud->invertedList(x, pointCloud->k-1) " << pointCloud->GetIndexFromDistanceList(pointCounter, -1) << "\n";
        std::cout << "pX  " << currentPoint.x() << " " << currentPoint.y() << " " << currentPoint.z() << "\n";
        std::cout << "pXK " << fathestPoint.x() << " " << fathestPoint.y() << " " << fathestPoint.z() << "\n";
        std::cout << "pq  " << pointToFathestPoint.x() << " " << pointToFathestPoint.y() << " " << pointToFathestPoint.z() << ", " << pointToFathestPoint.norm() << "\n";

        std::cout << "max_ed2  " << max_ed2 << "\n";
        std::cout << "log(mKInfluence) " << std::log(K_INFLUENCE) << " " << (-max_ed2 / std::log(K_INFLUENCE)) << "\n";
        std::cout << "pointCloud->sigma " << pointStandardDeviation << "\n";*/
        
        // Run the neighborhood
        // Note: the distance list does not include the vertex itself. Its size is points.size()-1 
        for (int neighborCounter = 0; neighborCounter < points.size()-1; ++neighborCounter)
        {
            //std::cout << "neighborCounter " << neighborCounter << "\n";
            // Displace in the normal direction, so additive noise could be softened
            const Point* p = pointCloud->GetPointFromDistanceList(pointCounter, neighborCounter);
            double rand = random_gauss->m_Random();
            Eigen::Vector3d randomNormal = p->GetNormal() * rand * sigmaN;
            Eigen::Vector3d neighborPoint = p->GetPosition();
            Eigen::Vector3d pointToNeighborPoint = neighborPoint + randomNormal - currentPoint;
            // Allign the normal direction with the eigensystem
            Eigen::Vector3d pq_alligned = *eigenVectors * pointToNeighborPoint;

            // Creating the spherical coordinates correspondence
            Eigen::Vector3d sphericalCoordCorresp;
            sphericalCoordCorresp(0) = pq_alligned.norm(); // rho

            double norm2pq = std::sqrt(pq_alligned(0) * pq_alligned(0) + pq_alligned(1) * pq_alligned(1));
            double tan_spherical = pq_alligned(2) / norm2pq;

            if (std::abs(norm2pq) < 0.0000077)
                sphericalCoordCorresp(1) = std::numbers::pi / 2.0; // phi
            else
                sphericalCoordCorresp(1) = std::atan2(pq_alligned(2), norm2pq); //phi

            if (std::abs(pq_alligned(0)) < 0.0000077)
                sphericalCoordCorresp(2) = std::numbers::pi / 2.0; // theta
            else
                sphericalCoordCorresp(2) = std::atan2(pq_alligned(1), pq_alligned(0)); //theta


            double d = std::tan(ellipsoid_angle);
            double beta = std::atan2(2.0 * (d*d) * tan_spherical, (d*d) - (tan_spherical * tan_spherical));

            double cosbeta = std::cos(beta);

            // Compute the vote vector
            Eigen::Vector3d voteVector;
            voteVector(0) = std::cos(sphericalCoordCorresp(2)) * cosbeta;
            voteVector(1) = std::sin(sphericalCoordCorresp(2)) * cosbeta;
            voteVector(2) = std::sin(beta);

            /*std::cout << "rand " << rand << "\n";
            std::cout << "pqAux " << randomNormal(0) << " " << randomNormal(1) << " " << randomNormal(2) << "\n";
            std::cout << "pq    " << pointToNeighborPoint(0) << " " << pointToNeighborPoint(1) << " " << pointToNeighborPoint(2) << "\n";
            std::cout << "pql   " << pq_alligned(0) << " " << pq_alligned(1) << " " << pq_alligned(2) << "\n";
            std::cout << "esf   " << sphericalCoordCorresp(0) << " " << sphericalCoordCorresp(1) << " " << sphericalCoordCorresp(2) << "\n";
            std::cout << "norm2pq " << norm2pq << "\n";
            std::cout << "tanesf  " << tan_spherical << "\n";
            std::cout << "d       " << d << " beta " << beta << "\n";
            std::cout << "cosbeta " << cosbeta << "\n";
            std::cout << "vn   " << voteVector(0) << " " << voteVector(1) << " " << voteVector(2) << "\n";*/

            // Constrain the conexion angle. Only values bellow may account.
            if (limit_angle == std::numbers::pi / 2.0 || std::abs(tan_spherical) <= std::abs(std::tan(limit_angle)))  //restringe o angulo de conexao
            {
                // Compute the Elliptical distance
                // If 45o, dist2 = rho cos(phi) (1 + tg² (z /|| x² + y²||))
                double s = sphericalCoordCorresp(0) * std::cos(sphericalCoordCorresp(1)) * std::pow(1.0 + (2.0 - (1.0 / SQR(d))) * SQR(tan(sphericalCoordCorresp(1))), SQR(d) / (2.0 * SQR(d) - 1.0));
                double f = std::exp(- ( (s / pointStandardDeviation)* (s / pointStandardDeviation)) );

                //Bring back from the eigensystem
                Eigen::Matrix3d transposeMatrix = eigenVectors->transpose();
                Eigen::Vector3d vnl = transposeMatrix * voteVector;
                Eigen::Matrix3d t = vnl * vnl.transpose();
                for (int j = 0; j < 9; j++) 
                    tensor_sum(j, pointCloud->GetIndexFromDistanceList(pointCounter, neighborCounter)) += f * t(j);
                /*for (int j = 0; j < 9; j++)
                    std::cout <<"ts "<< tensor_sum(j, pointCloud->GetIndexFromDistanceList(pointCounter, neighborCounter)) << " " << t(j) << "\n";

                std::cout << "Influence s " << s << "\n";
                std::cout << "Decay f " << f << "\n";
                std::cout << "vnl   " << vnl(0) << " " << vnl(1) << " " << vnl(2) << "\n";
                std::cout << "t     " << t(0) << " " << t(1) << " " << t(2) << "\n";*/
            }
        }
    }

    /*std::cout << "\ntensorsum.col(i).norm()\n";
    for (int i = 0; i < tensor_sum.rows(); i++)
        std::cout << i << ": " << tensor_sum.col(i).norm() << " ";
    std::cout << "\n";*/

    double l2 = 0.0;
    double maxl2 = 0.0;
    for (int pointCounter = 0; pointCounter < points.size(); ++pointCounter)
    {
        l2 = tensor_sum.col(pointCounter).norm();
        if (l2 > maxl2) maxl2 = l2;

        Eigen::Matrix3d t;
        if (std::abs(l2) < 0.0000077) {
            t.setIdentity(3, 3);
        }
        else {
            t(0, 0) = tensor_sum(0, pointCounter);
            t(0, 1) = tensor_sum(1, pointCounter);
            t(0, 2) = tensor_sum(2, pointCounter);
            t(1, 0) = tensor_sum(3, pointCounter);
            t(1, 1) = tensor_sum(4, pointCounter);
            t(1, 2) = tensor_sum(5, pointCounter);
            t(2, 0) = tensor_sum(6, pointCounter);
            t(2, 1) = tensor_sum(7, pointCounter);
            t(2, 2) = tensor_sum(8, pointCounter);
        }
        // eigenvalues are sorted in increasing order
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(t);
        Eigen::Vector3d eval = solver.eigenvalues();
        //Eigen::Matrix3d evec = solver.eigenvectors();

        double planarCoefficient = std::real(((eval(1) - eval(0)) + (eval(1) - eval(0))) / (eval(0) + eval(1) + eval(2)));

        /*std::cout << "t" << "\n";
        std::cout << t << "\n";
        std::cout << "eval " << eval(0) << " " << eval(1) << " " << eval(2) << "\n";
        std::cout << "cp " << planarCoefficient << "\n";*/
        const auto& points = pointCloud->GetPoints();
        if (!regularization || planarCoefficient > points.at(pointCounter)->GetTensorPlanarCoefficient())
        {
            points.at(pointCounter)->SetTensor(t);
            /*std::cout << "updateTensor" << "\n";
            std::cout << *points.at(pointCounter)->GetTensorMatrix() << "\n";
            std::cout << "eigenvalues " << points.at(pointCounter)->GetTensorEigenValues()->transpose() << "\n";
            std::cout << "eigenvector\n" << *points.at(pointCounter)->GetTensorEigenVectors() << "\n";*/

            /*file << std::fixed << std::setprecision(20) << "pointCounter " << pointCounter << " updateTensor" << "\n";
            file << t << "\n";
            file << *points.at(pointCounter)->GetTensorMatrix() << "\n";
            file << "eigenvalues " << points.at(pointCounter)->GetTensorEigenValues()->transpose() << "\n";
            file << "eigenvector\n" << *points.at(pointCounter)->GetTensorEigenVectors() << "\n";*/
        }
        //pointCloud->list[x]->normal = pointCloud->list[x]->tensor.eigenVectors.row(2).real();
        
        //file << "n: " << points.at(pointCounter)->GetNormal().transpose() << "\n";
        //std::cout << "enter a number\n";
        //int nada;  std::cin >> nada;
    } 
    //file.close();
}
