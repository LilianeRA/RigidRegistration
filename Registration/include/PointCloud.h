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
		
		PointCloud* Copy() const;

		void SetType(TYPE type);
		void SetColor(const Eigen::Vector3d &color);
		void Build(int skipstep);
		void SaveInput(const std::string& testpath);
		void SetCTSF_DistanceList(); // for SWC

		void ApplyTransformation(const Eigen::Affine3d &transformation);

		int GetTotalPoints() const;
		const Point* GetFarthestPoint(int pointIndex) const;
		const Point* GetPointFromDistanceList(int pointIndex, int listIndex) const;
		const int GetIndexFromDistanceList(int pointIndex, int listIndex) const;
		const std::vector<Point*>& GetPoints() const;
		bool IsCTSF_DistanceListSet() const;


		/*int createHole(double radius, int index);

		const std::vector<Point*>& getNormalizedPoints() const;
		const std::vector<Point*>& getDownscaledPoints() const;

		Graphics* getGraphics(char point_or_hole) const;

    protected:*/

    private:
		PointCloud();

		TYPE type = TYPE::ERROR;
		std::string filename{ "" };
		std::string filepath{ "" };
		Eigen::Vector3d color;

		std::vector<Point*> originalVertices;
		std::vector<std::vector<std::pair<int, double>>> distanceList; // list of distances between all vertices to all vertices. Index and distance
		std::vector<std::vector<std::pair<int, double>>> PureCTSF_distanceList; // for SWC
		/*std::vector<Point*> downscaledVertices;
		std::vector<Point*> normalizedVertices;
		std::vector<Point*> drawholes;
		double scale; // of normalization

		Graphics *pointgraphics;
		Graphics *holegraphics;*/

		int precision = 15;
		int skipstep = 0;
		
		void SetDistanceList();
		void Read();
		/*void downscale();
		void normalize();*/
};

#endif // POINTCLOUD_H