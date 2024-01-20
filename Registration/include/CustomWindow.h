#ifndef CUSTOMWINDOW_H
#define CUSTOMWINDOW_H

#include "WindowGLFW.h"
#include "MethodsData.h"
#include "RigidRegistration.h"

class CustomWindow : public WindowGLFW
{
	public:
		CustomWindow(bool bidimensional, const std::string& title, int width = 1080, int height = 720);
		virtual ~CustomWindow();

		void SetSourcePointCloud(const PointCloud* pointcloud, const glm::vec3& color);
		void SetTargetPointCloud(const PointCloud* pointcloud, const glm::vec3& color);

		void SetRegistration(RigidRegistration *registration);
	protected:
		virtual void SetCustomWindow() override;
		virtual void CustomDraw() override;
		virtual void CustomShutdown() override;

	private:
		bool showSrcPointCloud = true;
		bool showTgtPointCloud = true;
		std::string sourcePointCloudName{ "" };
		std::string targetPointCloudName{ "" };
		const PointCloud* sourceCloud; // for reseting the method

		//std::vector<DrawableSpheres*> mOtherSpheres;
		DrawableSpheres* originalSourceSpheres = nullptr;
		DrawableSpheres* sourceSpheres = nullptr;
		DrawableSpheres* targetSpheres = nullptr;

		static int methodChoice;
		static int matchChoice;
		static int estimationChoice;
		std::vector<std::pair<std::string, bool>> methodConfig;
		std::vector<std::pair<std::string, bool>> matchConfig;
		std::vector<std::pair<std::string, bool>> estimationConfig;
		float ctsf_percentage = 0.0f; // 0 to 100 for the GUI. Remember to divide by 100 before passing it to the registration method.

		RigidRegistration *registration = nullptr;

		void GetActiveMethod(const MethodsData *data);
		void SetActiveMethod();
		void TransformSourceSpheres(const Eigen::Affine3d& transformation);
		void ResetSourceCloud();
		void ColorCorrespondences(const glm::dvec3 &correspColor);
};
#endif // CUSTOMWINDOW_H
