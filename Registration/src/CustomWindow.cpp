#include "CustomWindow.h"
#include "imgui.h"

int CustomWindow::matchChoice = 0;
int CustomWindow::estimationChoice = 0;

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

void CustomWindow::SetActiveMethod(const MethodsData* data)
{
	MethodsData::MODE mode;
	MethodsData::METHOD method;
	MethodsData::MATCH match;
	MethodsData::ESTIMATION estimation;
	data->getActiveMethod(mode, method, match, estimation);

	methodConfig.clear();
	methodConfig.push_back(std::pair<std::string, bool>("ICP", (method == MethodsData::METHOD::ICP ? true : false)));
	methodConfig.push_back(std::pair<std::string, bool>("SWC", (method == MethodsData::METHOD::SWC ? true : false)));
	//methodConfig.push_back(std::pair<std::string, bool>("GMM", (method == MethodsData::METHOD::GMM ? true : false)));
	//methodConfig.push_back(std::pair<std::string, bool>("Super 4PCS", (method == MethodsData::METHOD::SUPER4PCS ? true : false)));
	//methodConfig.push_back(std::pair<std::string, bool>("Sparse ICP", (method == MethodsData::METHOD::SPARSEICP ? true : false)));


	matchConfig.clear();
	matchConfig.push_back(std::pair<std::string, bool>("ICP ", (match == MethodsData::MATCH::ICP ? true : false))); // you can't have two radio buttons with the same name
	//matchConfig.push_back(std::pair<std::string, bool>("GMM", (match == MethodsData::MATCH::GMM ? true : false)));
	matchConfig.push_back(std::pair<std::string, bool>("CTSF", (match == MethodsData::MATCH::CTSF ? true : false)));
	matchConfig.push_back(std::pair<std::string, bool>("LIEDIR", (match == MethodsData::MATCH::LIEDIR ? true : false)));
	matchConfig.push_back(std::pair<std::string, bool>("LIEIND", (match == MethodsData::MATCH::LIEIND ? true : false)));
	//matchConfig.push_back(std::pair<std::string, bool>("SUPER4PCS", (match == MethodsData::MATCH::SUPER4PCS ? true : false)));


	estimationConfig.clear();
	estimationConfig.push_back(std::pair<std::string, bool>("ICP", (estimation == MethodsData::ESTIMATION::ICP ? true : false)));
	estimationConfig.push_back(std::pair<std::string, bool>("SWC", (estimation == MethodsData::ESTIMATION::SWC ? true : false)));
	//estimationConfig.push_back(std::pair<std::string, bool>("GMM", (estimation == MethodsData::ESTIMATION::GMM ? true : false)));
	//estimationConfig.push_back(std::pair<std::string, bool>("SPARSEICP", (estimation == MethodsData::ESTIMATION::SPARSEICP ? true : false)));
	//estimationConfig.push_back(std::pair<std::string, bool>("SUPER4PCS", (estimation == MethodsData::ESTIMATION::SUPER4PCS ? true : false)));
}

void CustomWindow::SetPointCloud(const PointCloud *pointcloud, const std::string &pointCloudName, const glm::vec3 &color) 
{
	if (!pointcloud) return;

	const auto& points = pointcloud->GetPoints();
	DrawableSpheres* ds = new DrawableSpheres(pointCloudName);
	for (const auto& pt : points)
	{
		const Eigen::Vector3d &eigen_p = pt->GetPosition();
		glm::vec3 p{ eigen_p.x(), eigen_p.y(), eigen_p.z()};
		ds->PushSphere(p, color, 0.01);
	}
	mOtherSpheres.push_back(ds);
}

void CustomWindow::SetCustomWindow()
{
	ImGui::Begin("Custom Configuration");

	ImGui::Text("Show Point Cloud:"); 
	ImGui::SameLine();
	ImGui::Checkbox("Source", &showSrcPointCloud);
	ImGui::SameLine();
	ImGui::Checkbox("Target", &showTgtPointCloud);

	ImGui::Text("Method configuration ----------");

	/*int method_v_button = 0;
	ImGui::Text("Method:");
	for (const auto& method : methodConfig)
	{
		ImGui::SameLine();
		ImGui::RadioButton(method.first.c_str(), &methodChoice, method_v_button++);
	}*/


	/*enum Mode
	{
		Mode_Copy,
		Mode_Move,
		Mode_Swap
	};
	static int mode = 0;
	if (ImGui::RadioButton("Copy", mode == Mode_Copy)) { mode = Mode_Copy; } ImGui::SameLine();
	if (ImGui::RadioButton("Move", mode == Mode_Move)) { mode = Mode_Move; } ImGui::SameLine();
	if (ImGui::RadioButton("Swap", mode == Mode_Swap)) { mode = Mode_Swap; }*/
	// use with e.g. if (RadioButton("one", my_value==1)) { my_value = 1; }
	int estimation_v_button = 0;
	ImGui::Text("Estimation:");
	for (const auto& estimation : estimationConfig)
	{
		ImGui::SameLine();
		ImGui::RadioButton(estimation.first.c_str(), &estimationChoice, estimation_v_button++);
	}
	int match_v_button = 0;
	ImGui::Text("Match:");
	for (const auto& match : matchConfig)
	{
		ImGui::SameLine();
		ImGui::RadioButton(match.first.c_str(), &matchChoice, match_v_button++);
	}

	if (ImGui::Button("Start Registration"))
	{
		std::cout << "Starting: " << estimationConfig.at(estimationChoice).first << "-" << matchConfig.at(matchChoice).first << std::endl;
	}
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


