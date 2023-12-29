#include <random>

#include "pch.h"
#include "happly.h"
#include "Random.h"
#include "RandomVector.h"
#include "RandomGaussian.h"

#define NUM_SEEDS 20

unsigned int get_size_without_outliers(float outlier_percentage, unsigned int cols)
{
    return static_cast<unsigned int>(std::ceil(float(cols)/(1.0f+outlier_percentage)));
}

void read_ply(const std::string &filename, unsigned int downscale,
              Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    if(downscale == 0)
    {
        PRINT_ERROR("read_ply::Error: downscale can't be 0\n");
        exit(-1);
    }
    // Construct a data object by reading from file
    happly::PLYData plyIn(filename);

    // Get data from the object
    std::vector<float> x_s = plyIn.getElement("vertex").getProperty<float>("x");
    std::vector<float> y_s = plyIn.getElement("vertex").getProperty<float>("y");
    std::vector<float> z_s = plyIn.getElement("vertex").getProperty<float>("z");

    if(x_s.size() == y_s.size() && x_s.size() == z_s.size())
    {
        LOG("cols: "<<static_cast<int>(x_s.size()/downscale));
        point_cloud = Eigen::Matrix<double, 3, Eigen::Dynamic>(3,static_cast<int>(x_s.size()/downscale));
        for(Eigen::Index i = 0, j = 0; i < x_s.size(); i++)
        {
            if((i+1)%downscale == 0)
            {
                point_cloud.col(j) << x_s.at(i), y_s.at(i), z_s.at(i);
                j++;
            }
        }
    }
    else
    {
        PRINT_ERROR("read_ply::Error: coordinates doesn't have the same size!");
        LOG("size x "<<x_s.size());
        LOG("size y "<<y_s.size());
        LOG("size z "<<z_s.size());
        x_s.clear();
        y_s.clear();
        z_s.clear();
        exit(-1);
    }
    x_s.clear();
    y_s.clear();
    z_s.clear();
}

float normalize(Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    Eigen::Vector3d max_pt{-INT_MAX,-INT_MAX,-INT_MAX};
    Eigen::Vector3d min_pt{ INT_MAX, INT_MAX, INT_MAX};

    for(Eigen::Index i = 0; i < point_cloud.cols(); i++)
    {
        if(min_pt.x() > point_cloud.col(i).x()) min_pt.x() = point_cloud.col(i).x();
        if(min_pt.y() > point_cloud.col(i).y()) min_pt.y() = point_cloud.col(i).y();
        if(min_pt.z() > point_cloud.col(i).z()) min_pt.z() = point_cloud.col(i).z();

        if(max_pt.x() < point_cloud.col(i).x()) max_pt.x() = point_cloud.col(i).x();
        if(max_pt.y() < point_cloud.col(i).y()) max_pt.y() = point_cloud.col(i).y();
        if(max_pt.z() < point_cloud.col(i).z()) max_pt.z() = point_cloud.col(i).z();
    }
    LOG("min: "<<min_pt.x()<<" "<<min_pt.y()<<" "<<min_pt.z());
    LOG("max: "<<max_pt.x()<<" "<<max_pt.y()<<" "<<max_pt.z());
    Eigen::Vector3d sub = max_pt-min_pt;
    float scale = static_cast<float>(1.0/(std::max(std::max(sub.x(), sub.y()), sub.z())));
    LOG("scale: "<<scale);
    for(Eigen::Index i = 0; i < point_cloud.cols(); i++)
    {
        point_cloud.col(i) = scale*(point_cloud.col(i)-min_pt);
    }
    //LOG(point_cloud.col(0));
    return scale;
}

void apply_transformation(const Eigen::Quaterniond &q,
                          const Eigen::Vector3d &t,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d centroid{0.0,0.0,0.0};
    for(Eigen::Index i = 0; i < point_cloud.cols(); i++)
    {
        centroid += point_cloud.col(i);
    }
    centroid = centroid/point_cloud.cols();

    for(Eigen::Index i = 0; i < point_cloud.cols(); i++)
    {
        point_cloud.col(i) = R*(point_cloud.col(i)-centroid) + t + centroid;
    }
    //LOG(point_cloud.col(0));
}

unsigned int create_hole(Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud, float radius, unsigned int hole_index)
{
    /// 703 880
    unsigned int ind = INT_MAX;
    if(hole_index == 0) ind = 703;
    else if(hole_index == 1) ind = 880-83;
    else ind = rand()%point_cloud.cols();

    Eigen::Vector3d center{point_cloud.col(ind)};
    unsigned int deleted = 0;
    // Identifying the points to be deleted
    for(unsigned int i = 0; i < point_cloud.cols(); i++)
    {
        if(point_cloud.col(i).x() == INT_MAX) deleted++;
        if((center-point_cloud.col(i)).norm() < radius) // radius*scale
        {
            point_cloud.col(i) << INT_MAX,INT_MAX,INT_MAX;
            deleted++;
        }
    }
    LOG("deleted: "<<deleted<<" "<<point_cloud.cols()-deleted);
    // saving a new point cloud without the INT_MAX points
    Eigen::Matrix<double, 3, Eigen::Dynamic> point_cloud_holes(3, point_cloud.cols()-deleted);
    for(unsigned int i = 0, j = 0; i < point_cloud.cols(); i++)
    {
        if(point_cloud.col(i).x() == INT_MAX) continue;
        point_cloud_holes.col(j) = point_cloud.col(i);
        j++;
    }
    point_cloud = point_cloud_holes;
    return ind;
}

void additive_noise(float noise_percentage, const std::array<int, NUM_SEEDS> &seeds,
                    unsigned int initial_seed, Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    RandomVector *random_vec = new RandomVector(); //generate the displacement vectors on the additive noise step
    RandomGaussian *random_gauss = new RandomGaussian(); //generate the size of the vectors on the additive noise step

    random_vec->m_SetSeed(seeds[initial_seed], seeds[initial_seed+1]);
    random_gauss->m_SetSeed(seeds[initial_seed+2], seeds[initial_seed+3]);

    Eigen::Vector3d rand_unit_sphere_vec;

    for(unsigned int i = 0; i < point_cloud.cols(); i++)
    {
        random_vec->m_DoubleRandomVec(rand_unit_sphere_vec[0], rand_unit_sphere_vec[1], rand_unit_sphere_vec[2]);
        rand_unit_sphere_vec.normalize(); // random unit vec direction. Forms a sphere in S^2 of radius 1 if you plot all possible points
        //LOG(rand_unit_sphere_vec.norm());
        rand_unit_sphere_vec = rand_unit_sphere_vec*random_gauss->m_Random()*noise_percentage; // choosing the size from 0 to noise_percentage
        point_cloud.col(i) += rand_unit_sphere_vec; // adding noise to the position
    }
}

void outliers(float outlier_percentage, const std::array<int, NUM_SEEDS> &seeds,
              unsigned int initial_seed, Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    // Generate extra points inside a 3D sphere around the point cloud.
    // The points must be outside the point cloud's bounding box
    // and the sphere must be centered at the cloud's centroid.

    // Creating new point cloud with room for outliers
    unsigned int total_points = point_cloud.cols()*(1.0f+outlier_percentage);
    Eigen::Matrix<double, 3, Eigen::Dynamic> point_cloud_outliers(3, total_points);
    point_cloud_outliers.setZero(3, total_points);

    // Get the cloud centroid and bounding box
    Eigen::Vector3d centroid{0.0,0.0,0.0};
    Eigen::Vector3d min_pt{ INT_MAX, INT_MAX, INT_MAX};
    Eigen::Vector3d max_pt{-INT_MAX,-INT_MAX,-INT_MAX};
    for(unsigned int i = 0; i < point_cloud.cols(); i++)
    {
        centroid += point_cloud.col(i); // computing centroid
        point_cloud_outliers.col(i) = point_cloud.col(i); // copying the original points

        min_pt.x() = std::min(min_pt.x(),point_cloud.col(i).x());
        min_pt.y() = std::min(min_pt.x(),point_cloud.col(i).y());
        min_pt.z() = std::min(min_pt.x(),point_cloud.col(i).z());

        max_pt.x() = std::max(max_pt.x(), point_cloud.col(i).x());
        max_pt.y() = std::max(max_pt.y(), point_cloud.col(i).y());
        max_pt.z() = std::max(max_pt.z(), point_cloud.col(i).z());
    }
    centroid = centroid/point_cloud.cols();

    std::uniform_real_distribution<double> ud{-1.0,1.0}; // for random unitary vector

    // the point cloud is normalized. Max side will be 1.0
    //double sphere_size = std::max(std::max(sub.x(), sub.y()), sub.z())*3.0;
    double sphere_size = 2.0; // you can change this for any value above 1.0
    Eigen::Vector3d sphere_center{0.5,0.5,0.5}; // the center of normalized point cloud's BB will always be 0.5

    unsigned int total_outliers = total_points-point_cloud.cols();
    unsigned int counter_outliers = 0;


    Random *random_number = new Random();
    random_number->m_SetSeed(seeds[initial_seed]);

    Eigen::Vector3d outlier;
    while(counter_outliers < total_outliers) // select random outliers
    {
        outlier = Eigen::Vector3d{random_number->m_DoubleRandom(), random_number->m_DoubleRandom(), random_number->m_DoubleRandom()};
        outlier.normalize();                                // random unit directions. Forms a sphere shell.
        outlier = outlier*sphere_size*random_number->m_DoubleRandom() + centroid;   // sphere_size*ud(gen) is for filling the 3D sphere
        // Is it inside the bounding box? If yes, reject.
        if(outlier.x() < max_pt.x() && outlier.y() < max_pt.y() && outlier.z() < max_pt.z() &&
                outlier.x() > min_pt.x() && outlier.y() > min_pt.y() && outlier.z() > min_pt.z())
        {
            continue;
        }
        // valid position: inside the sphere, outside the BB
        point_cloud_outliers.col(point_cloud.cols()+counter_outliers) = outlier;
        counter_outliers++;
        //LOG(counter_outliers);
    }

    point_cloud.resize(point_cloud_outliers.rows(), point_cloud_outliers.cols());
    point_cloud = point_cloud_outliers; // point_cloud_outliers will be free as soon as the function ends
}

void save_ply(const std::string &base_dir, const std::string &filename,
              const Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    std::vector<float> x_s, y_s, z_s;
    x_s.reserve(point_cloud.cols());
    y_s.reserve(point_cloud.cols());
    z_s.reserve(point_cloud.cols());
    for(unsigned int i = 0; i < point_cloud.cols(); i++)
    {
        x_s.emplace_back(point_cloud.col(i).x());
        y_s.emplace_back(point_cloud.col(i).y());
        z_s.emplace_back(point_cloud.col(i).z());
    }
    // Create an empty object
    happly::PLYData plyOut;

    // Add elements
    plyOut.addElement("vertex", x_s.size());

    // Add properties to those elements
    plyOut.getElement("vertex").addProperty<float>("x", x_s);
    plyOut.getElement("vertex").addProperty<float>("y", y_s);
    plyOut.getElement("vertex").addProperty<float>("z", z_s);

    // Write the object to file
    fs::path filepath = fs::current_path()/fs::path(base_dir)/fs::path("point_clouds");
    if(!fs::exists(filepath))
        fs::create_directory(filepath);

    filepath = filepath/fs::path(filename);
    if(!fs::exists(filepath))
    {
        plyOut.write(filepath.string(), happly::DataFormat::ASCII);
    } // not saving the same file twice

    x_s.clear();
    y_s.clear();
    z_s.clear();
}

void build_new_pointcloud(Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud,
						  const std::string &base_dir,
                          std::string &filename, unsigned int downscale, unsigned int rotation, char translation, unsigned int holes,
                          float radius, float noise, int noise_seed, float outlier, int outlier_seed,
                          const std::array<int, NUM_SEEDS> &seeds)
{
    Eigen::Quaterniond q45{0.955586, 0.00548449, -0.294635, -0.0038555};
    Eigen::Quaterniond q90{0.706009, 0.000335889, -0.708202,  0.000602459};
    Eigen::Vector3d t45 {-0.0520211, -0.000383981, -0.0109223};
    Eigen::Vector3d t90 { 2.20761e-05, -3.34606e-05, -7.20881e-05};

    float scale = normalize(point_cloud);

    if(rotation == 45)
    {
        if(translation == '1')
        {
            apply_transformation(q45, t45, point_cloud);
        }
        else
        {
            if(translation == '2')
            {
                apply_transformation(q45, t90, point_cloud);
            }
        }
    }
    else
    {
        if(rotation == 90)
        {
            if(translation == '2')
            {
                apply_transformation(q90, t90, point_cloud);
            }
            else
            {
                if(translation == '1')
                {
                    apply_transformation(q90, t45, point_cloud);
                }
            }
        }
    }


    for(unsigned int i = 0; i < holes; i++)
    {
        unsigned int index1 = create_hole(point_cloud, radius*scale, i);
        LOG("hole index: "<< index1);
    }

    if(noise > 0.0f)
    {
        additive_noise(noise, seeds, noise_seed, point_cloud);
    }

    if(outlier > 0.0f)
    {
        outliers(outlier, seeds, outlier_seed, point_cloud);
    }

    std::string t{translation};
    std::string n = std::to_string((int)(noise*100));
    std::string o = std::to_string((int)(outlier*100));
    std::string rad{"0"};
    if(holes > 0) rad = std::to_string(radius);

    rad = rad.substr(0,4);

    filename +=  "_down"+std::to_string(downscale);
    filename +=   "_deg"+std::to_string(rotation);
    filename +=     "_t"+t;
    filename += "_holes"+std::to_string(holes);
    filename +=   "_rad"+rad;
    filename += "_noise"+n;
    filename +=   "_out"+o;
    filename += ".ply";

    LOG("Saving at: "<<filename);
    save_ply(base_dir, filename, point_cloud);
}

int main(int args, char** argv)
{
    if(args < 11)
    {
        std::cout<< "Error: Missing arguments\n";
        std::cout<< "Usage: input_dir ply_filename downscale rotation translation holes hole_radius noise outlier seed\n";
        std::cout<< "../ bun000 45 0 0 0 0 0 0 1\n"; // only downscale
        std::cout<< "../ bun000 45 45 1 2 0.03 1 5 1\n"; // rotate, translate, 2 holes, 1% noise 5% outlier
        std::cout<< "../ bun000 45 90 2 0 0.0 3 20 1\n"; // rotate, translate, no holes, 3% noise 20% outlier
    }
    // Getting the arguments
    std::string input_dir{argv[1]};
    std::string filename{argv[2]};
    unsigned int downscale = atoi(argv[3]);
    unsigned int rotation  = atoi(argv[4]);
    char translation    = argv[5][0];
    unsigned int holes  = atoi(argv[6]);
    float radius        = ( (atoi(argv[6]) > 0) ? atof(argv[7]) : 0.0f);
    float noise         = atof(argv[8])/100.0f;
    float outlier       = atof(argv[9])/100.0f;
    int seed            = atoi(argv[10]);

    std::array<int, NUM_SEEDS> seeds;
    LOG("ARGS");
    LOG("input_dir   "<<input_dir);
    LOG("filename    "<<filename);
    LOG("downscale   "<<downscale);
    LOG("rotation    "<<rotation);
    LOG("translation "<<translation);
    LOG("holes       "<<holes);
    LOG("radius      "<<radius);
    LOG("noise       "<<noise);
    LOG("outlier     "<<outlier);
    LOG("seeds       "<<seeds.size());
    LOG("\n");
    LOG("Current path: "<<fs::current_path());

    // seeds computed by a random number generator
    Random *rn = new Random();
    rn->m_SetSeed(seed);
    for(unsigned int i = 0; i < seeds.size(); i++)
    {
        seeds.at(i) = rn->m_IntRandom(0, INT_MAX);
        LOG(i<<" seeds[i] "<<seeds[i]);
    }

    // reading the input point cloud
    std::string ply_path = (fs::current_path()/fs::path(input_dir)/fs::path(filename)).string();
    if (!fs::is_regular_file(ply_path+".ply"))
    {
        LOG("Error: input point cloud not found: " << ply_path);
        return -1;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> point_cloud_src; // a.k.a. data
    Eigen::Matrix<double, 3, Eigen::Dynamic> point_cloud_tgt; // a.k.a. model

    LOG("Reading point cloud...");
    read_ply(ply_path+".ply", downscale, point_cloud_src);
    read_ply(ply_path+".ply", downscale, point_cloud_tgt);

    // Source and Target point clouds are affected if there is noise, outliers or holes.
    // The noise applied to the src cloud is not the same applied to the tgt cloud.
    // Same is valid for the outliers.
    // Tgt clouds do not rotate nor translate
    std::string filename_src = filename+"_src";
    std::string filename_tgt = filename+"_tgt";
    LOG("Building source and target point clouds...");
    build_new_pointcloud(point_cloud_src, input_dir, filename_src, downscale, rotation, translation,
                         holes, radius, noise, 7, outlier, 2, seeds);
    build_new_pointcloud(point_cloud_tgt, input_dir, filename_tgt, downscale, 0, '0',
                         holes, radius, noise, 3, outlier, 1, seeds);

    LOG("Done");
    return 0;
}


/**
void generateAdditiveNoise(int initialSeed, std::vector<int> &seeds)
{
    RandomVector *randVec = new RandomVector(); //generate the displacement vectors on the additive noise step
    RandomGaussian *randGauss = new RandomGaussian(); //generate the size of the vectors on the additive noise step

    randVec->m_SetSeed(seeds[initialSeed], seeds[initialSeed+1]);
    randGauss->m_SetSeed(seeds[initialSeed+2], seeds[initialSeed+3]);

    VECTOR3d temp;
    double scaletemp;
    for (unsigned int i = 0; i < 10; i++){
        randVec->m_DoubleRandomVec(temp[0], temp[1], temp[2]);
        scaletemp = randGauss->m_Random();
        std::cout<<i<<": "<<temp[0]<<" "<<temp[1]<<" "<<temp[2]<<std::endl;
        std::cout<<i<<": "<<scaletemp<<std::endl;
    }
}

void generateOutliers(int initialSeed, std::vector<int> &seeds)
{
    Random *randomNumber = new Random();
    randomNumber->m_SetSeed(seeds[initialSeed]);
    for (int i = 0; i < 10; i++){
        auto x__ = randomNumber->m_DoubleRandom();
        auto y__ = randomNumber->m_DoubleRandom();
        auto z__ = randomNumber->m_DoubleRandom();
        std::cout<<i<<": "<<x__<<" "<<y__<<" "<<z__<<std::endl;
    }
}

void recreate_random_numbers()
{
    std::vector<int> seeds(20, 0);
    Random *rn = new Random();
    rn->m_SetSeed(1);
    for (int i = 0; i < 20; i++){
        seeds[i] = rn->m_IntRandom(0, INT_MAX);
        std::cout<<i<<" seeds[i] "<<seeds[i]<<std::endl;
    }
    generateOutliers(1, seeds);
    generateOutliers(2, seeds);
    generateAdditiveNoise(3, seeds);
    generateAdditiveNoise(7, seeds);
}












void additive_noise(float noise_percentage, std::mt19937 &gen, Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    // Generate random numbers with Gaussian distribution with
    // 0 mean and 1.0 as standard-deviation N(0, 1).
    std::normal_distribution<double> nd{0,1}; // for random magnitude [0,1]
    std::uniform_real_distribution<double> ud{0,1}; // for random unitary vector

    Eigen::Vector3d rand_unit_sphere_vec;

    for(unsigned int i = 0; i < point_cloud.cols(); i++)
    {
        rand_unit_sphere_vec = Eigen::Vector3d{ud(gen), ud(gen), ud(gen)}; // random vec
        rand_unit_sphere_vec.normalize(); // random unit vec direction. Forms a sphere in S^2 of radius 1 if you plot all possible points
        //LOG(rand_unit_sphere_vec.norm());
        rand_unit_sphere_vec = rand_unit_sphere_vec*nd(gen)*noise_percentage; // choosing the size from 0 to noise_percentage
        point_cloud.col(i) += rand_unit_sphere_vec; // adding noise to the position
    }
}

void outliers(float outlier_percentage, std::mt19937 &gen, Eigen::Matrix<double, 3, Eigen::Dynamic> &point_cloud)
{
    // Generate extra points inside a 3D sphere around the point cloud.
    // The points must be outside the point cloud's bounding box
    // and the sphere must be centered at the cloud's centroid.

    // Creating new point cloud with room for outliers
    unsigned int total_points = point_cloud.cols()*(1.0f+outlier_percentage);
    Eigen::Matrix<double, 3, Eigen::Dynamic> point_cloud_outliers(3, total_points);
    point_cloud_outliers.setZero(3, total_points);

    // Get the cloud centroid and bounding box
    Eigen::Vector3d centroid{0.0,0.0,0.0};
    Eigen::Vector3d min_pt{ INT_MAX, INT_MAX, INT_MAX};
    Eigen::Vector3d max_pt{-INT_MAX,-INT_MAX,-INT_MAX};
    for(unsigned int i = 0; i < point_cloud.cols(); i++)
    {
        centroid += point_cloud.col(i); // computing centroid
        point_cloud_outliers.col(i) = point_cloud.col(i); // copying the original points

        min_pt.x() = std::min(min_pt.x(),point_cloud.col(i).x());
        min_pt.y() = std::min(min_pt.x(),point_cloud.col(i).y());
        min_pt.z() = std::min(min_pt.x(),point_cloud.col(i).z());

        max_pt.x() = std::max(max_pt.x(), point_cloud.col(i).x());
        max_pt.y() = std::max(max_pt.y(), point_cloud.col(i).y());
        max_pt.z() = std::max(max_pt.z(), point_cloud.col(i).z());
    }
    centroid = centroid/point_cloud.cols();

    std::uniform_real_distribution<double> ud{-1.0,1.0}; // for random unitary vector

    // the point cloud is normalized. Max side will be 1.0
    //double sphere_size = std::max(std::max(sub.x(), sub.y()), sub.z())*3.0;
    double sphere_size = 2.0; // you can change this for any value above 1.0
    Eigen::Vector3d sphere_center{0.5,0.5,0.5}; // the center of normalized point cloud's BB will always be 0.5

    unsigned int total_outliers = total_points-point_cloud.cols();
    unsigned int counter_outliers = 0;

    Eigen::Vector3d outlier;
    while(counter_outliers < total_outliers) // select random outliers
    {
        outlier = Eigen::Vector3d{ud(gen), ud(gen), ud(gen)};
        outlier.normalize();                                // random unit directions. Forms a sphere shell.
        outlier = outlier*sphere_size*ud(gen) + centroid;   // sphere_size*ud(gen) is for filling the 3D sphere
        // Is it inside the bounding box? If yes, reject.
        if(outlier.x() < max_pt.x() && outlier.y() < max_pt.y() && outlier.z() < max_pt.z() &&
                outlier.x() > min_pt.x() && outlier.y() > min_pt.y() && outlier.z() > min_pt.z())
        {
            continue;
        }
        // valid position: inside the sphere, outside the BB
        point_cloud_outliers.col(point_cloud.cols()+counter_outliers) = outlier;
        counter_outliers++;
        //LOG(counter_outliers);
    }

    point_cloud.resize(point_cloud_outliers.rows(), point_cloud_outliers.cols());
    point_cloud = point_cloud_outliers; // point_cloud_outliers will be free as soon as the function ends
}

*/


