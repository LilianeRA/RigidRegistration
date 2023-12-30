#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <string>
#include "Graphics.h"

class PointCloud
{
    public:
		enum class TYPE {SOURCE_DATA, TARGET_MODEL, ERROR};

        PointCloud(const std::string &filepath);
        virtual ~PointCloud();
		
		void setType(TYPE type);
		void setColor(const Eigen::Vector3d &color);
		void build(int skipstep);
		void saveInput(const std::string& testpath);
		int createHole(double radius, int index);

		Graphics* getGraphics(char point_or_hole);

    protected:

    private:

		TYPE type;
		std::string filename;
		std::string filepath;
		std::vector<Point*> originalVertices;
		std::vector<Point*> downscaledVertices;
		std::vector<Point*> normalizedVertices;
		std::vector<Point*> drawholes;
		Eigen::Vector3d color;
		double scale; // of normalization

		Graphics *pointgraphics;
		Graphics *holegraphics;

		int precision;
		int skipstep;
		void read();
		void downscale();
		void normalize();
};

#endif // POINTCLOUD_H