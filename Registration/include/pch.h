#ifndef PCH_H_INCLUDED
#define PCH_H_INCLUDED

#include <filesystem>
#include <iostream>
#include <iomanip>
#include <string>
#include <Eigen/Dense>

namespace fs = std::filesystem;

#define DEBUG 1

#if DEBUG == 1
#define LOG(x) std::cout<< x << std::endl
#define LOG15(x) std::cout<<std::setprecision(15)<< x << std::endl
#else
#define LOG(x)
#define LOG15(x)
#endif

#define PRINT_VEC3(a,v) std::cout<<std::setprecision(15)<<a<<v.x<<" "<<v.y<<" "<<v.z<< std::endl
#define PRINT_VEC4(a,v) std::cout<<std::setprecision(15)<<a<<v.x<<" "<<v.y<<" "<<v.z<<" "<<v.w<< std::endl
#define PRINT(x) std::cout<< x << std::endl
#define PAUSE(x) PRINT("Enter an integer to proceed: "); int x; std::cin>>x
#define EQUAL(x,y) (std::abs(x-y) < 1e-6)
#define SQR(x) (x*x)
#define PRINT_ERROR(x) PRINT("\n************"); PRINT(x); PRINT("************")
#define PRINT_WARNING(x) std::cout<< x << std::endl

#define EPSILON 1e-6

#endif // PCH_H_INCLUDED
