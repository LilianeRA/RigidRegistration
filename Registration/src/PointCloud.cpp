#include "PointCloud.h"
#include "DirHandler.h"
#include "Estimators.h"
#include "happly.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>      // std::setprecision

PointCloud::PointCloud()
{

}

PointCloud::PointCloud(const std::string& filepath)
{
    //ctor
	if(filepath.empty())
	{
		std::cout<<"Warning: Point cloud filename is empty. Aborting."<<std::endl;
		exit(EXIT_FAILURE);
	}

	this->filepath = filepath;
	this->filename = DirHandler::GetFileName(this->filepath);

	/*this->originalVertices.clear();
	this->downscaledVertices.clear();
	this->pointgraphics = new Graphics();
	this->holegraphics = new Graphics();*/

	this->color[0] = 0.0;
	this->color[1] = 0.0;
	this->color[2] = 0.0;
}

PointCloud::~PointCloud()
{
    //dtor
	for (unsigned int i = 0; originalVertices.size(); i++)
	{
		delete(originalVertices.at(i));
	}
	this->originalVertices.clear();
	
	/*
	for(unsigned int i = 0; downscaledVertices.size(); i++)
	{
		delete(downscaledVertices.at(i));
	}
	for(unsigned int i = 0; normalizedVertices.size(); i++)
	{
		delete(normalizedVertices.at(i));
	}
	
	this->downscaledVertices.clear();
	this->normalizedVertices.clear();
	this->drawholes.clear();*/
}

PointCloud* PointCloud::Copy() const
{
	PointCloud* copyPointCloud = new PointCloud();
	copyPointCloud->SetType(type);
	copyPointCloud->SetColor(color);

	std::copy(originalVertices.begin(), originalVertices.end(), std::back_inserter(copyPointCloud->originalVertices));
	std::copy(distanceList.begin(), distanceList.end(), std::back_inserter(copyPointCloud->distanceList));
	std::copy(PureCTSF_distanceList.begin(), PureCTSF_distanceList.end(), std::back_inserter(copyPointCloud->PureCTSF_distanceList));

	copyPointCloud->precision = precision;
	copyPointCloud->skipstep = skipstep;
	copyPointCloud->filename = filename;
	copyPointCloud->filepath = filepath;

	return copyPointCloud;
}

void PointCloud::SetType(TYPE type)
{
	this->type = type;
}

void PointCloud::SetColor(const Eigen::Vector3d &color)
{
	this->color = color;
}

void PointCloud::Build(int skipstep)
{
	if (skipstep < 0)
	{
		skipstep = 0;
		std::cout<<"Warning: skipstep invalid. Setting to 0."<<std::endl;
	}
	this->skipstep = skipstep;

	Read();

	/*if(this->skipstep >= 0)
		downscale();
	else
		std::cout<<"Warning: Not downscaling."<<std::endl;

	normalize();

	this->pointgraphics->setPoints(normalizedVertices);*/

	SetDistanceList();

}

void PointCloud::SetDistanceList()
{
	distanceList.clear();
	for (int i = 0; i < originalVertices.size(); ++i)
	{
		std::vector<std::pair<int, double>> distances;
		for (int j = 0; j < originalVertices.size(); ++j)
		{
			if (i == j) continue;
			double euclideanDistance = Point::EuclideanDistance(originalVertices.at(i), originalVertices.at(j), 0.0);
			distances.push_back(std::pair<int, double>(j, euclideanDistance));
		}
		// descending order
		std::stable_sort(distances.begin(), distances.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second < b.second; });
		distanceList.push_back(distances);
		//std::cout << i << "\t" << distances.front().first << " " << distances.front().second<< ", " << distances.back().first << " " << distances.back().second << "\n";
	}

	/*for (int i = 0; i < 5; i++)
	{
		std::cout << "*" << i << "*";
		for (int j = 0; j < 44; j++) 
		{
			std::cout << distanceList.at(i).at(j).first << " (" << distanceList.at(i).at(j).second << ")";
		}
		std::cout << std::endl;
	}*/

	/*for (int i = 0; i < 5; i++)
	{
		std::cout << "*" << i << "* ";
		for (int j = 0; j < 43; j++)
			std::cout << GetIndexFromDistanceList(i, j) << " ";
		std::cout << std::endl;
	}*/

}


void PointCloud::SetCTSF_DistanceList()
{
	PureCTSF_distanceList.clear();
	for (int i = 0; i < originalVertices.size(); ++i)
	{
		std::vector<std::pair<int, double>> distances;
		for (int j = 0; j < originalVertices.size(); ++j)
		{
			if (i == j) continue;
			double tensorDistance = Point::PureCTSF_TensorDistance(originalVertices.at(i), originalVertices.at(j));
			distances.push_back(std::pair<int, double>(j, tensorDistance));
		}
		// descending order
		std::stable_sort(distances.begin(), distances.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second < b.second; });
		PureCTSF_distanceList.push_back(distances);
		//std::cout << i << "\t" << distances.front().first << " " << distances.front().second<< ", " << distances.back().first << " " << distances.back().second << "\n";
	}
}
  
void PointCloud::Read()
{
	//std::cout << "read: " << this->filepath <<std::endl;
    if(this->filepath.find(".ply") == std::string::npos &&
       this->filepath.find(".dat") == std::string::npos)
    {
        std::cout<<"Error: input file not supported. Aborting."<<std::endl;
		std::cout<<"File: "<<this->filepath<<std::endl;
        exit(EXIT_FAILURE);
    }

	int counter = 0;
	int totalvertices = 0;
	bool startreading = false;
	std::string line;
    if(this->filepath.find(".ply") != std::string::npos)
    {
        std::ifstream plyfile;
        try
        {
            plyfile.open(this->filepath);
            if(plyfile.is_open())
            {
                while(getline(plyfile,line))
                {
                    //std::cout<<line<<'\n';
                    size_t found = line.find("element vertex");
                    if(found != std::string::npos)
                    {
                        totalvertices = std::atoi(line.substr(found+15).c_str());
                        //std::cout<<"Total of vertices "<<totalvertices<<std::endl;
                        continue;
                    }

                    found = line.find("end_header");
                    if(found != std::string::npos)
                    {
                        startreading = true;
                        continue;
                    }
                    if(startreading)
                    {
                        if(counter >= totalvertices) break;

                        std::stringstream ss(line);
                        double x, y, z;
                        ss >> x >> y >> z;
                        Point *vertex = new Point(x, y, z);
                        originalVertices.push_back(vertex);
                        counter++;
                    }
                }
                plyfile.close();
            }
        }
        catch(std::exception const& e)
        {
            std::cout<<"Error: "<< e.what()<<std::endl;
            std::cout<<"Ply file: "<< this->filepath<<std::endl;
        }
    }

    if(this->filepath.find(".dat") != std::string::npos)
    {
        std::ifstream datfile;
        try
        {
            datfile.open(this->filepath);
            if(datfile.is_open())
            {
                if(getline(datfile,line))
                {
                    std::stringstream ss(line);
                    ss >> totalvertices;
                    std::cout<<totalvertices<<std::endl;
                }
                while(getline(datfile,line))
                {
                    std::stringstream ss(line);
                    double x, y, z;
                    ss >> x >> y >> z;
                    Point *vertex = new Point(x, y, z);
                    originalVertices.push_back(vertex);
                }
            }
        }
        catch(std::exception const& e)
        {
            std::cout<<"Error: "<< e.what()<<std::endl;
            std::cout<<"Dat file: "<< this->filepath<<std::endl;
        }
    }
	//std::cout<<"Vertices: "<<originalVertices.size()<<std::endl;
}

/*void PointCloud::downscale()
{
	downscaledVertices.clear();
	for(unsigned int i = 0; i < originalVertices.size(); i++)
	{
	    if(this->skipstep > 0)
        {
            if((i+1)%this->skipstep == 0)
            {
                downscaledVertices.push_back(originalVertices.at(i));
            }
        }
        else downscaledVertices.push_back(originalVertices.at(i));
	}
	std::cout<<"Downscaled vertices: "<<downscaledVertices.size()<<std::endl;
}

void PointCloud::normalize()
{
	Eigen::Vector3d minp( 1000000.0, 1000000.0, 1000000.0);
	Eigen::Vector3d maxp(-1000000.0,-1000000.0,-1000000.0);
	for(unsigned int i = 0; i < downscaledVertices.size(); i++)
	{
		Point *p = downscaledVertices.at(i);
		Eigen::Vector3d pos = p->getPosition();
		if(pos[0] < minp[0]) minp[0] = pos[0];
		if(pos[1] < minp[1]) minp[1] = pos[1];
		if(pos[2] < minp[2]) minp[2] = pos[2];

		if(pos[0] > maxp[0]) maxp[0] = pos[0];
		if(pos[1] > maxp[1]) maxp[1] = pos[1];
		if(pos[2] > maxp[2]) maxp[2] = pos[2];
	}
    double scalex = 1.0/(maxp[0] - minp[0]);
    double scaley = 1.0/(maxp[1] - minp[1]);
    double scalez = 1.0/(maxp[2] - minp[2]);

    this->scale = (scalex < scaley ? (scalex < scalez ? scalex : scalez) : (scalez < scaley ? scalez : scaley));

	for(unsigned int i = 0; i < downscaledVertices.size(); i++)
	{
		Point *p = downscaledVertices.at(i);
		Eigen::Vector3d pos = p->getPosition();
		pos = (pos-minp)*scale;
		Point *np = new Point(pos[0], pos[1], pos[2]);
		np->setColor(this->color);
		np->setAsRemoved(p->isRemoved());
		normalizedVertices.push_back(np);
	}
}

int PointCloud::createHole(double radius, int index)
{
	if(index != -1)
	{
		if(index < 0 || index >= downscaledVertices.size())
		{
			std::cout<<"Warning: invalid index. Choosing a random one."<<std::endl;
			index = -1;
		}
		else
		{
			Eigen::Vector3d drawcenter = normalizedVertices.at(index)->getPosition();
			Point *p = new Point(drawcenter[0], drawcenter[1], drawcenter[2]);
			p->setColor(Eigen::Vector3d(0.0, 0.0, 0.7));
			p->setAsRemoved(false);
			drawholes.push_back(p);

			int totalRemoved = 0;
			Eigen::Vector3d center = downscaledVertices.at(index)->getPosition();
			for(unsigned int i = 0; i < downscaledVertices.size(); i++)
			{
				Eigen::Vector3d pos = downscaledVertices.at(i)->getPosition();
				Eigen::Vector3d sub = pos-center;
				if(sub.norm() < radius)
				{
					if(!downscaledVertices.at(i)->isRemoved()) totalRemoved++;
					downscaledVertices.at(i)->setAsRemoved(true);
					normalizedVertices.at(i)->setAsRemoved(true);
				}
			}
			std::cout<<"Removed "<<totalRemoved<<std::endl;

			if(this->drawholes.size() > 0)
			{
				this->holegraphics->setRadius(radius*this->scale*100);
				this->holegraphics->setPoints(drawholes);

			}
		}
	}
	if(index == -1)
	{
		/// get a random index
	}
	return index;
}

const std::vector<Point*>& PointCloud::getNormalizedPoints() const
{
	return normalizedVertices;
}

const std::vector<Point*>& PointCloud::getDownscaledPoints() const
{
	return downscaledVertices;
}

Graphics* PointCloud::getGraphics(char point_or_hole) const
{
	if(point_or_hole == 'p')
		return this->pointgraphics;
	if(point_or_hole == 'h')
		return this->holegraphics;
}*/

int PointCloud::GetTotalPoints() const
{
	return originalVertices.size();
}

const Point* PointCloud::GetFarthestPoint(int pointIndex) const
{
	if (pointIndex >= distanceList.size() || pointIndex < 0) return nullptr;
	return originalVertices.at(GetIndexFromDistanceList(pointIndex, -1));
}

const Point* PointCloud::GetPointFromDistanceList(int pointIndex, int listIndex) const
{
	int ind = GetIndexFromDistanceList(pointIndex, listIndex);
	if (ind < 0) return nullptr;
	return originalVertices.at(ind);
}

const int PointCloud::GetIndexFromDistanceList(int pointIndex, int listIndex) const
{
	if (pointIndex >= distanceList.size() || pointIndex < 0)
	{
		return -INT_MAX;
	}
	if (listIndex < 0)
	{
		return distanceList.at(pointIndex).back().first; // *************
	}
	else if (listIndex >= distanceList.at(pointIndex).size()) // comparing int with unsigned int
	{
		return -INT_MAX;
	}
	return distanceList.at(pointIndex).at(listIndex).first;
}

const std::vector<Point*>& PointCloud::GetPoints() const
{
	return originalVertices;
}

bool PointCloud::IsCTSF_DistanceListSet() const
{
	return (PureCTSF_distanceList.size() > 0 ? true : false);
}

void save_ply(const std::string &fullFilepath,
	const std::vector<Point*> point_cloud)
{
	std::vector<float> x_s, y_s, z_s;
	x_s.reserve(point_cloud.size());
	y_s.reserve(point_cloud.size());
	z_s.reserve(point_cloud.size());
	for (unsigned int i = 0; i < point_cloud.size(); i++)
	{
		const auto p = point_cloud.at(i)->GetPosition();
		x_s.emplace_back(p.x());
		y_s.emplace_back(p.y());
		z_s.emplace_back(p.z());
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
	if (!DirHandler::IsFile(fullFilepath))
	{
		plyOut.write(fullFilepath, happly::DataFormat::ASCII);
	} // not saving the same file twice

	x_s.clear();
	y_s.clear();
	z_s.clear();
}

void PointCloud::SaveInput(const std::string & testpath)
{
	if (originalVertices.size() > 0 )
	{
		std::string fullpathOrig{ DirHandler::JoinPaths(testpath, this->filename) };
		if (this->type == TYPE::SOURCE_DATA)
		{
			fullpathOrig = fullpathOrig + "-source-orig.ply";
		}
		else if (this->type == TYPE::TARGET_MODEL)
		{
			fullpathOrig = fullpathOrig + "-target-orig.ply";
		}
		save_ply(fullpathOrig, originalVertices);
	}
	/*if (downscaledVertices.size() > 0)
	{
		std::string fullpathOrig{ DirHandler::JoinPaths(testpath, this->filename) };
		if (this->type == TYPE::SOURCE_DATA)
		{
			fullpathOrig = fullpathOrig + "-source-down.ply";
		}
		else if (this->type == TYPE::TARGET_MODEL)
		{
			fullpathOrig = fullpathOrig + "-target-down.ply";
		}
		save_ply(fullpathOrig, downscaledVertices);
	}
	if (normalizedVertices.size() > 0 )
	{
		std::string fullpathOrig{ DirHandler::JoinPaths(testpath, this->filename) };
		if (this->type == TYPE::SOURCE_DATA)
		{
			fullpathOrig = fullpathOrig + "-source-norm.ply";
		}
		else if (this->type == TYPE::TARGET_MODEL)
		{
			fullpathOrig = fullpathOrig + "-target-norm.ply";
		}
		save_ply(fullpathOrig, normalizedVertices);
	}*/

	/*if (originalVertices.size() == 0 ||
	   downscaledVertices.size() == 0 ||
	   normalizedVertices.size() == 0)
	{
		std::cout<<"Warning: not saving empty vertices list. Returning."<<std::endl;
		return;
	}
	//std::cout<<testname<<std::endl;
	std::string fullpathOrig(testname+"/"+this->filename);
	std::string fullpathDown(testname+"/"+this->filename);
	std::string fullpathNorm(testname+"/"+this->filename);
	if(this->type == TYPE::SOURCE_DATA)
	{
		fullpathOrig = fullpathOrig + "-source-orig.dat";
		fullpathDown = fullpathDown + "-source-down.dat";
		fullpathNorm = fullpathNorm + "-source-norm.dat";
	}
	else if(this->type == TYPE::TARGET_MODEL)
	{
		fullpathOrig = fullpathOrig + "-target-orig.dat";
		fullpathDown = fullpathDown + "-target-down.dat";
		fullpathNorm = fullpathNorm + "-target-norm.dat";
	}

	std::ofstream datfile(fullpathOrig);
	datfile<<std::fixed<<std::setprecision(this->precision)<<originalVertices.size()<<"\n";
	for(unsigned int i = 0; i < originalVertices.size(); i++)
	{
		Point *p = originalVertices.at(i);
		Eigen::Vector3d pos = p->getPosition();
		datfile<<pos[0]<<" ";
		datfile<<pos[1]<<" ";
		datfile<<pos[2]<<"\n";
	}
	datfile.close();

	datfile.open(fullpathDown);
	datfile<<downscaledVertices.size()<<"\n";
	for(unsigned int i = 0; i < downscaledVertices.size(); i++)
	{
		Point *p = downscaledVertices.at(i);
		Eigen::Vector3d pos = p->getPosition();
		datfile<<pos[0]<<" ";
		datfile<<pos[1]<<" ";
		datfile<<pos[2]<<" ";
		datfile<<p->isRemoved()<<"\n";
	}
	datfile.close();

	datfile.open(fullpathNorm);
	datfile<<normalizedVertices.size()<<"\n";
	for(unsigned int i = 0; i < normalizedVertices.size(); i++)
	{
		Point *p = normalizedVertices.at(i);
		Eigen::Vector3d pos = p->getPosition();
		datfile<<pos[0]<<" ";
		datfile<<pos[1]<<" ";
		datfile<<pos[2]<<" ";
		datfile<<p->isRemoved()<<"\n";
	}
	datfile.close();*/
}

void PointCloud::ApplyTransformation(const Eigen::Affine3d& transformation)
{
	const Eigen::Vector3d centroid = Estimators::ComputeCenteroid(GetPoints());

	//int i = 0;
	for (Point* p : originalVertices)
	{
		p->Translate(-centroid);
		p->Rotate(transformation);
		p->Translate(centroid);

		/*if (i < 10)
			std::cout << i << ": " << p->GetPosition().transpose() << std::endl;
		i++;*/
		
		const Eigen::Matrix3d* tensorMatrix = p->GetTensorMatrix();
		if (tensorMatrix)
		{
			Eigen::Matrix3d t = (transformation.rotation() * (*tensorMatrix)) * transformation.rotation().transpose();
			p->SetTensor(t);
		}
	}
}

