#pragma warning(disable: 4710) // Suppress 'function ... not inlined' for Release builds
#pragma warning(disable: 4711) // Suppress 'function ... selected fpr automatic inline expansion' for Release builds
#pragma warning(disable: 4514) // Suppress '... unreferenced inline function has been removed'
#pragma warning(push, 3)       // Set warning levels to a quieter level for the STL

#include <iostream>
#include <filesystem>
#include <Eigen/Dense>
#include <fstream>

#pragma warning(pop)           // Restore warning levels for our code

#include "pch.h"
#include "CommandLineParser.h"
#include "DirHandler.h"
#include "RigidRegistration.h"


// For getting the current directory
#ifdef _WIN32
#include <direct.h>
// MSDN recommends against using getcwd & chdir names
#define cwd _getcwd
#define cd _chdir
namespace fs = std::filesystem;
#else
#include "unistd.h"
#include <stdio.h>
#define cwd getcwd
#define cd chdir
#include <dirent.h> // DIR
#endif


/**
./RigidRegistration -sourcemesh=bunny000.ply -targetmesh=bunny045.ply -totalholes=1 -holeradius=0.03 -datadir E:\GIT\RigidRegistration\BuildInput\point_clouds
./RigidRegistration -sourcemesh=bunny000.dat -targetmesh=bunny045.dat -totalholes=2 -holeradius=0.03 -downscalestep=0

.\RigidRegistration.exe -sourcemesh="bun000_src_down45_deg0_t0_holes0_rad0_noise0_out0.ply" -targetmesh="bun000_src_down45_deg45_t1_holes2_rad0.03_noise1_out5.ply" -datadir="E:\GIT\RigidRegistration\BuildInput\point_clouds"
.\RigidRegistration.exe -sourcemesh="bun000.ply" -targetmesh="bun000.ply" -totalholes=2 -holeradius=0.03 -noise=1 -outliers=5 -datadir="E:\GIT\RigidRegistration\BuildInput\point_clouds"
**/

int main(int argc, char const** argv)
{
    /**
    -mode=meshbatch     // batch processing of two meshes
    -mode=meshview      // view of two meshes
        -downscalepoints
        -noise -outliers -overlap
        -hole -holeradius
        -sourcemesh -targetmesh
    -mode=videobatch    // batch processing of a video sequence
    -mode=videoview     // view of a video sequence
        -frameskip
        -datadir (a "depth" and "rgb" folder must be inside)

    -testname -sequencename -threads
    -trimming -executionnumber

    -useGT -gtfilename -downscalestep -datadir
    -alphacut=45 -alphaellipse=45 -kctsf=10(nao usado)
    -sigmaN=0 -stepK=0.1

    // --- ICP
    -method=ICP -match=ICP    -estimation=ICP
    -method=ICP -match=CTSF   -estimation=ICP
    -method=ICP -match=LIEDIR -estimation=ICP
    -method=ICP -match=LIEIND -estimation=ICP

    // --- Sparse ICP
    -method=SICP -match=ICP  -estimation=SICP
    -method=SICP -match=CTSF -estimation=SICP -kmodel/kdata={5 10 50 7swc5}

    // --- SWC -kmodel/kdata={5 10 50 75}
    -method=SWC -match=ICP    -estimation=ICP
    -method=SWC -match=CTSF   -estimation=ICP
    -method=SWC -match=LIEDIR -estimation=ICP
    -method=SWC -match=LIEIND -estimation=ICP

    // --- GMM
    -method=GMM -match=GMM -estimation=GMM

    // --- Super 4PCS
    -method=S4PCS -match=S4PCS -estimation=S4PCS
    **/


    std::string maindir(DirHandler::GetCurrentDir());
    size_t found = maindir.find("build");
    if (found != std::string::npos) 
        maindir = maindir.substr(0, found - 1);
    std::cout << maindir << "\n";

    std::string keys =
        "{help h  |     | show help message}"   // optional, show help optional
        "{mode    | meshbatch | execution mode }"
        "{inputdir  | . | full path }"
        "{outputdir | . | full path }"
        "{method  | ICP | string }"
        "{match   | ICP | string }"
        "{estimation     | ICP | string }"
        "{downscalestep  | 0   | integer }" // or 45
        "{downscalepoints|     | integer }"
        "{noise      | 0.0     | percentage }"
        "{outliers   | 0.0     | percentage }"
        "{overlap    | 0.0     | }"
        "{totalholes | 0       | integer }"
        "{holeradius | 0.0     | real }"
        "{testname   | test    | string }"
        "{threads    | 4       | integer }"
        "{trimming   | 1.0     | percentage }"
        "{executionnumber | 1  | integer }"
        "{useGT      | false   | boolean }"
        "{gtfilename | groundtruth.txt | full path }"
        "{sourcemesh | bunny000.ply | full path }"
        "{targetmesh | bunny045.ply | full path }"
        ;

    CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    /// Required arguments
    std::string mode = parser.get<std::string>("mode"); // read model (mandatory, error if not present)

    int downscalestep = 0; // old image sampling
    int downscalepoints = -1; // select n random numbers of the point cloud

    /// For mesh mode
    double holeradius = -1.0;
    double noise = -1.0; // percentage
    double outliers = -1.0; // percentage
    double overlap = -1.0;
    int    totalholes = 0;
    std::string sourcemesh("");
    std::string targetmesh("");

    std::string inputdir = parser.get<std::string>("inputdir");   // Can't be empty
    if (!DirHandler::IsDirectory(inputdir))
    {
        PRINT_ERROR("Error: input directory path is not a valid path: " << inputdir);
        return -1;
    }
    
    std::string outputdir = parser.get<std::string>("outputdir"); // Can't be empty
    if (!DirHandler::IsDirectory(outputdir))
    {
        DirHandler::CreateDir(outputdir);
    }
    if (outputdir.compare(".") == 0) // if it is the current dir, then copy the absolute path from maindir
        outputdir = maindir;

    if (mode.find("meshbatch") != std::string::npos || mode.find("meshview") != std::string::npos)
    {
        noise = parser.get<double>("noise"); 
        outliers = parser.get<double>("outliers");
        overlap = parser.get<double>("overlap");
        totalholes = parser.get<int>("totalholes");
        sourcemesh = parser.get<std::string>("sourcemesh");
        targetmesh = parser.get<std::string>("targetmesh");

        if (totalholes < 0)
        {
            PRINT_WARNING("Warning: given 'total of holes' is negative. Considering that there is no hole.");
            totalholes = 0;
        }
        if (totalholes > 0)
        {
            holeradius = parser.get<double>("holeradius");
        }

        // Validation
        if (noise < 0.0 || noise > 100.0)
        {
            PRINT_ERROR("Error: noise is a percentage. Choose a number between 0 and 100. Given: " << std::to_string(noise));
            return -1;
        }
        if (outliers < 0.0 || outliers > 100.0)
        {
            PRINT_ERROR("Error: outliers is a percentage. Choose a number between 0 and 100. Given: " << std::to_string(outliers));
            return -1;
        }


        if (!DirHandler::IsFile(DirHandler::JoinPaths(inputdir, sourcemesh)))
        {
            PRINT_ERROR("Error: source mesh path is not a valid file: " << DirHandler::JoinPaths(inputdir, sourcemesh));
            return -1;
        }
        if (!DirHandler::IsFile(DirHandler::JoinPaths(inputdir, targetmesh)))
        {
            PRINT_ERROR("Error: target mesh path is not a valid file: " << DirHandler::JoinPaths(inputdir, targetmesh));
            return -1;
        }

    }

    if (mode.find("videobatch") != std::string::npos || mode.find("videoview") != std::string::npos)
    {

    }

    if (parser.has("downscalestep"))
    {
        downscalestep = parser.get<int>("downscalestep");
        if (parser.has("downscalepoints"))
        {
            PRINT_WARNING("Warning: downscalepoints will not be considered");
        }
        // Validation
        if (downscalestep < 0)
        {
            PRINT_ERROR("Error: downscale step must be non negative. Given: " << std::to_string(downscalestep));
            return -1;
        }
    }
    
    if (!parser.has("downscalestep") && parser.has("downscalepoints"))
    {
        downscalepoints = parser.get<int>("downscalepoints");
        // Validation
        if (downscalepoints < 0)
        {
            PRINT_ERROR("Error: downscale points must be non negative. Given: " << std::to_string(downscalepoints));
            return -1;
        }
    }
    

    std::string method = parser.get<std::string>("method");         // ICP SICP SWC S4PCS GMM
    std::string match = parser.get<std::string>("match");           // ICP CTSF S4PCS GMM LIEDIR LIEIND
    std::string estimation = parser.get<std::string>("estimation"); // ICP SICP S4PCS GMM

    std::string testname = parser.get<std::string>("testname"); // Can't be empty
    if (testname.empty())
    {
        PRINT_ERROR("Error: test name must not be empty");
        return -1;
    }

    int threads = 4; // standard
    if (parser.has("threads"))
    {
        threads = parser.get<int>("threads");
    }

    //double trimming     = parser.get<double>("trimming");
    //int executionnumber = parser.get<int>("executionnumber");
    bool useGT          = parser.get<bool>("useGT");

    std::string gtfilename("");
    if (useGT)
    {
        gtfilename = parser.get<std::string>("gtfilename");
        if (gtfilename.empty())
        {
            std::cout << "Error: gtfilename empty\n";
            return -1;
        }
        if (mode.find("meshbatch") != std::string::npos || mode.find("meshview") != std::string::npos)
        {
            std::ifstream infile(maindir + "/" + gtfilename); // txt
            //std::cout<<maindir+"/"+gtfilename<<std::endl;
            std::string line;
            Eigen::Quaterniond quaternion(0, 0, 0, 1);
            Eigen::Translation3d translation(0, 0, 0);
            while (std::getline(infile, line))
            {
                if (line.find("#") != std::string::npos) continue;
                if (line.find("qx=") != std::string::npos) quaternion.x() = atof(line.substr(3).c_str());
                if (line.find("qy=") != std::string::npos) quaternion.y() = atof(line.substr(3).c_str());
                if (line.find("qz=") != std::string::npos) quaternion.z() = atof(line.substr(3).c_str());
                if (line.find("qw=") != std::string::npos) quaternion.w() = atof(line.substr(3).c_str());

                if (line.find("tx=") != std::string::npos) translation.x() = atof(line.substr(3).c_str());
                if (line.find("ty=") != std::string::npos) translation.y() = atof(line.substr(3).c_str());
                if (line.find("tz=") != std::string::npos) translation.z() = atof(line.substr(3).c_str());
            }
            //std::cout<<quaternion.x()<<" "<<quaternion.y()<<" "<<quaternion.z()<<" "<<quaternion.w()<<std::endl;
            //std::cout<<translation.x()<<" "<<translation.y()<<" "<<translation.z()<<std::endl;
            infile.close();
        }

    }

    const double alphacut = 45;
    const double alphaellipse = 45;
    //const double kctsf = 10; //(nao usado)
    const double sigmaN = 0;
    const double stepK = 0.1;

    if (!parser.check()) {
        parser.printErrors();
        return -1;
    }

    
    PRINT("\n******************************");
    PRINT("mode       " << mode);
    PRINT("inputdir   " << inputdir);
    PRINT("outputdir  " << outputdir);
    PRINT("method     " << method);
    PRINT("match      " << match);
    PRINT("estimation " << estimation);
    PRINT("downscalestep " << std::to_string(downscalestep));
    PRINT("downscalepoints " << std::to_string(downscalepoints));
    PRINT("noise      " << std::to_string(noise));
    PRINT("outliers   " << std::to_string(outliers));
    PRINT("overlap    " << std::to_string(overlap));
    PRINT("totalholes " << std::to_string(totalholes));
    PRINT("holeradius " << std::to_string(holeradius));
    PRINT("testname   " << testname);
    PRINT("threads    " << std::to_string(threads));
    //PRINT("trimming   " << std::to_string(trimming));
    //PRINT("executionnumber " << std::to_string(executionnumber));
    PRINT("useGT      " << std::to_string(useGT));
    PRINT("gtfilename " << gtfilename);
    PRINT("sourcemesh " << sourcemesh);
    PRINT("targetmesh " << targetmesh);
    PRINT("******************************\n");


    RigidRegistration* RR = new RigidRegistration(maindir, inputdir, outputdir, testname, threads);
    RR->SetMode(mode);
    RR->SetMethod(method, match, estimation);
    RR->SetGTfile(gtfilename);
    RR->SetTensorParameters(alphacut, alphaellipse, sigmaN);
    RR->SetPointClouds(sourcemesh, targetmesh, downscalestep, totalholes, holeradius);
    RR->SaveParameters();
    RR->Run();

    //SparseICP(argc, argv);
    





    return 0;
}
