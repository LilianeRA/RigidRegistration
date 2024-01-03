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

		const std::vector<Point*>& getPoints() const;

		/*int createHole(double radius, int index);

		const std::vector<Point*>& getNormalizedPoints() const;
		const std::vector<Point*>& getDownscaledPoints() const;

		Graphics* getGraphics(char point_or_hole) const;

    protected:*/

    private:

		TYPE type = TYPE::ERROR;
		std::string filename{ "" };
		std::string filepath{ "" };
		Eigen::Vector3d color;

		std::vector<Point*> originalVertices;
		/*std::vector<Point*> downscaledVertices;
		std::vector<Point*> normalizedVertices;
		std::vector<Point*> drawholes;
		double scale; // of normalization

		Graphics *pointgraphics;
		Graphics *holegraphics;*/

		int precision = 15;
		int skipstep = 0;

		void read();
		/*void downscale();
		void normalize();*/
};

#endif // POINTCLOUD_H