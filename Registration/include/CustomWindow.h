#ifndef CUSTOMWINDOW_H
#define CUSTOMWINDOW_H

#include "WindowGLFW.h"
#include "PointCloud.h"

class CustomWindow : public WindowGLFW
{
	public:
		CustomWindow(bool bidimensional, const std::string& title, int width = 1080, int height = 720);
		virtual ~CustomWindow();

		void SetSourcePointCloud(const PointCloud* pointcloud, const glm::vec3& color);
		void SetTargetPointCloud(const PointCloud* pointcloud, const glm::vec3& color);

	protected:
		virtual void SetCustomWindow() override;
		virtual void CustomDraw() override;
		virtual void CustomShutdown() override;

	private:
		bool showSrcPointCloud = true;
		bool showTgtPointCloud = true;
		std::string sourcePointCloudName{ "" };
		std::string targetPointCloudName{ "" };

		std::vector<DrawableSpheres*> mOtherSpheres;


		void SetPointCloud(const PointCloud* pointcloud, const std::string& pointCloudName, const glm::vec3& color);
};
#endif // CUSTOMWINDOW_H
