#include "CustomWindow.h"
#include "imgui.h"

int CustomWindow::methodChoice = 0;
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
	if (!pointcloud) return;

	sourcePointCloudName = "Source";
	//SetPointCloud(pointcloud, sourcePointCloudName, color);

	const auto& points = pointcloud->GetPoints();
	originalSourceSpheres = new DrawableSpheres(sourcePointCloudName);
	for (const auto& pt : points)
	{
		const Eigen::Vector3d& eigen_p = pt->GetPosition();
		glm::vec3 p{ eigen_p.x(), eigen_p.y(), eigen_p.z() };
		originalSourceSpheres->PushSphere(p, color, 0.01);
	}
	ResetSourceCloud();
}

void CustomWindow::SetTargetPointCloud(const PointCloud* pointcloud, const glm::vec3& color)
{
	if (!pointcloud) return;

	targetPointCloudName = "Target";
	//SetPointCloud(pointcloud, targetPointCloudName, color);

	const auto& points = pointcloud->GetPoints();
	targetSpheres = new DrawableSpheres(targetPointCloudName);
	for (const auto& pt : points)
	{
		const Eigen::Vector3d& eigen_p = pt->GetPosition();
		glm::vec3 p{ eigen_p.x(), eigen_p.y(), eigen_p.z() };
		targetSpheres->PushSphere(p, color, 0.01);
	}
}

void CustomWindow::GetActiveMethod(const MethodsData* data)
{
	MethodsData::MODE mode;
	MethodsData::METHOD method;
	MethodsData::MATCH match;
	MethodsData::ESTIMATION estimation;
	data->getActiveMethod(mode, method, match, estimation);

	methodConfig.clear();
	methodConfig.push_back(std::pair<std::string, bool>("ICP  ", (method == MethodsData::METHOD::ICP ? true : false)));
	methodConfig.push_back(std::pair<std::string, bool>("SWC  ", (method == MethodsData::METHOD::SWC ? true : false)));
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

	for (int index = 0; index < methodConfig.size(); index++)
	{
		const auto& method = methodConfig.at(index);
		if (method.second)
		{
			methodChoice = index;
		}
	}
	for (int index = 0; index < matchConfig.size(); index++)
	{
		const auto& match = matchConfig.at(index);
		if (match.second)
		{
			matchChoice = index;
		}
	}
	for (int index = 0; index < estimationConfig.size(); index++)
	{
		const auto& estimation = estimationConfig.at(index);
		if (estimation.second)
		{
			estimationChoice = index;
		}
	}
	this->ctsf_percentage = data->GetTensorNeighborPercentage() * 100.0;
}

void CustomWindow::SetActiveMethod()
{
	MethodsData* data = this->registration->GetMethodsData();
	std::string method = methodConfig.at(methodChoice).first; 
	std::string match = matchConfig.at(matchChoice).first; 
	std::string estimation = estimationConfig.at(estimationChoice).first; 
	// remove the blank spaces
	method.erase(std::remove_if(method.begin(), method.end(), [](unsigned char x) { return std::isspace(x); }), method.end());
	match.erase (std::remove_if( match.begin(),  match.end(), [](unsigned char x) { return std::isspace(x); }), match.end());
	estimation.erase(std::remove_if(estimation.begin(), estimation.end(), [](unsigned char x) { return std::isspace(x); }), estimation.end());

	std::cout << "SetActiveMethod() " << method << ", " << match << ", " << estimation << ", "<< ctsf_percentage <<"\n";
	data->setMethod(method, match, estimation, ctsf_percentage/100.0, true);
	registration->Reset();
}

void CustomWindow::ResetSourceCloud()
{
	if (sourceSpheres)
	{
		delete sourceSpheres;
		sourceSpheres = nullptr;
	}
	sourceSpheres = originalSourceSpheres->Copy();
}

void CustomWindow::SetRegistration(RigidRegistration* registration)
{
	this->registration = registration;
	GetActiveMethod(registration->GetMethodsData());
}

void CustomWindow::TransformSourceSpheres(const Eigen::Affine3d &transformation)
{
	/*for (auto* spheres : mOtherSpheres)
	{
		if (pointCloudName.compare(spheres->GetName()) == 0)
		{
			glm::dvec3 translation;
			glm::dmat3 transf;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
					transf[i][j] = transformation(j, i);
				translation[i] = transformation(i, 3);
			}
			glm::dvec3 centroid = spheres->ComputeCentroid();
			for (int sphereIndex = 0; sphereIndex < spheres->GetTotalSpheres(); ++sphereIndex)
			{
				spheres->RotateSpherePosition(sphereIndex, transf, centroid);
				spheres->TranslateSpherePosition(sphereIndex, translation);
			}
			break;
		}
	}*/

	glm::dvec3 translation;
	glm::dmat3 transf;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			transf[i][j] = transformation(j, i); // yes, it's the transpose
		translation[i] = transformation(i, 3);
	}
	glm::dvec3 centroid = sourceSpheres->ComputeCentroid();
	for (int sphereIndex = 0; sphereIndex < sourceSpheres->GetTotalSpheres(); ++sphereIndex)
	{
		sourceSpheres->RotateSpherePosition(sphereIndex, transf, centroid);
		sourceSpheres->TranslateSpherePosition(sphereIndex, translation);
	}
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

	int method_v_button = 0;
	ImGui::Text("Method:");
	for (const auto& method : methodConfig)
	{
		ImGui::SameLine();
		ImGui::RadioButton(method.first.c_str(), &methodChoice, method_v_button++);
	}
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

	static int iterations = 0;
	static int currentIteration = 0;
	ImGui::SliderFloat("CTSF percentage", &ctsf_percentage, 0.0f, 100.0f, "%.0f");
	if (ImGui::Button("Start Registration"))
	{
		std::cout << "Starting: " << estimationConfig.at(estimationChoice).first << "-" << matchConfig.at(matchChoice).first << std::endl;

		ResetSourceCloud();
		SetActiveMethod();
		transformations.clear();
		transformations = registration->Run();
		iterations = transformations.size()+1;
		// updating the visualization
		for (const auto& t : transformations)
		{
			TransformSourceSpheres(t);
		}
		ColorCorrespondences(glm::dvec3(0.0,0.5,0.0), registration->GetCorrespondentPoints());
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset point clouds"))
	{
		transformations.clear();
		ResetSourceCloud(); // so you can see it before restarting
		iterations = 0;
		currentIteration = 0;
	}
	ImGui::SliderInt("Iterations", &currentIteration, 0, iterations);
	if (ImGui::Button("View iteration")) 
	{
		ResetSourceCloud(); // so you can see it before restarting
		for (int index = 0; index < currentIteration; index++)
			TransformSourceSpheres(transformations.at(index));
		//std::vector<bool> correspondences;

		//ColorCorrespondences(glm::dvec3(0.0, 0.5, 0.0), correspondences);
	}
	ImGui::End();
}

void CustomWindow::ColorCorrespondences(const glm::dvec3& correspColor, 
	const std::vector<bool> & correspondences)
{
	for (int index = 0; index < correspondences.size(); ++index)
	{
		if (correspondences.at(index))
		{
			sourceSpheres->SetSphereColor(index, correspColor);
		}
	}
}

void CustomWindow::CustomDraw()
{
	/*for (const auto ds : mOtherSpheres)
	{
		if (!showSrcPointCloud && sourcePointCloudName.compare(ds->GetName()) == 0) continue;
		if (!showTgtPointCloud && targetPointCloudName.compare(ds->GetName()) == 0) continue;

		ds->Draw(mLightPos, mLightColor);
	}*/
	if (showSrcPointCloud && sourceSpheres) sourceSpheres->Draw(mLightPos, mLightColor);
	if (showTgtPointCloud && targetSpheres) targetSpheres->Draw(mLightPos, mLightColor);
}

void CustomWindow::CustomShutdown()
{
	/*for (auto ds : mOtherSpheres)
		delete ds;
	mOtherSpheres.clear();*/
	delete sourceSpheres;
	delete targetSpheres;
}


