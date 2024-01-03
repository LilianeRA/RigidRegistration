#include "CustomWindow.h"
#include "imgui.h"

CustomWindow::CustomWindow(bool bidimensional, const std::string& title, int width, int height)
	: WindowGLFW(bidimensional, title, width, height)
{
}

CustomWindow::~CustomWindow()
{
	
}

void CustomWindow::SetSourcePointCloud(const PointCloud* pointcloud, const glm::vec3& color)
{
	sourcePointCloudName = "Source";
	SetPointCloud(pointcloud, sourcePointCloudName, color);
}
void CustomWindow::SetTargetPointCloud(const PointCloud* pointcloud, const glm::vec3& color)
{
	targetPointCloudName = "Target";
	SetPointCloud(pointcloud, targetPointCloudName, color);
}
void CustomWindow::SetPointCloud(const PointCloud *pointcloud, const std::string &pointCloudName, const glm::vec3 &color) 
{
	if (!pointcloud) return;

	const auto& points = pointcloud->getPoints();
	DrawableSpheres* ds = new DrawableSpheres(pointCloudName);
	for (const auto& pt : points)
	{
		const Eigen::Vector3d &eigen_p = pt->getPosition();
		glm::vec3 p{ eigen_p.x(), eigen_p.y(), eigen_p.z()};
		ds->PushSphere(p, color, 0.01);
	}
	mOtherSpheres.push_back(ds);
}

void CustomWindow::SetCustomWindow()
{
	ImGui::Begin("Custom Configuration");
	ImGui::Checkbox("Show source point cloud:", &showSrcPointCloud);
	ImGui::Checkbox("Show target point cloud:", &showTgtPointCloud);
	ImGui::End();
}

void CustomWindow::CustomDraw()
{
	for (const auto ds : mOtherSpheres)
	{
		if (!showSrcPointCloud && sourcePointCloudName.compare(ds->GetName()) == 0) continue;
		if (!showTgtPointCloud && targetPointCloudName.compare(ds->GetName()) == 0) continue;

		ds->Draw(mLightPos, mLightColor);
	}
}

void CustomWindow::CustomShutdown()
{
	for (auto ds : mOtherSpheres)
		delete ds;
	mOtherSpheres.clear();
}


