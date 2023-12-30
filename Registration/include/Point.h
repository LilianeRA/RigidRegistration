#ifndef POINT_H
#define POINT_H

#include <Eigen/Dense>

class Point
{
    public:
        Point(double x, double y, double z);
        virtual ~Point();

		Eigen::Vector3d getPosition();
		Eigen::Vector3d getColor();
		bool isRemoved();
		void setAsRemoved(bool removed);
		void setColor(const Eigen::Vector3d &color);
		void translate(const Eigen::Vector3d &translation);
		void rotate(const Eigen::Affine3d &rotation);		

    protected:

    private:
		Eigen::Vector3d position;
		Eigen::Vector3d color;
		Eigen::Vector3d normal;
		bool removed;
};

#endif // POINT_H