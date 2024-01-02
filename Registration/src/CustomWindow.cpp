#include "CustomWindow.h"
#include "imgui.h"

CustomWindow::CustomWindow(bool bidimensional, const std::string& title, int width, int height) 
	: WindowGLFW(bidimensional, title, width, height)
{
}

CustomWindow::~CustomWindow()
{
	
}

void CustomWindow::SetCustomWindow()
{
	ImGui::Begin("Custom Configuration");
    // custom checkboxes
	ImGui::End();
}

void CustomWindow::CustomDraw()
{
	for (const auto ds : mOtherSpheres)
	{
		ds->Draw(mLightPos, mLightColor);
	}
}

void CustomWindow::CustomShutdown()
{
	for (auto ds : mOtherSpheres)
		delete ds;
}


void CustomWindow::AppendDrawableSphere(DrawableSpheres* ds)
{
	if (ds)
	{
		mOtherSpheres.push_back(ds);
		std::cout << "mOtherSpheres " << mOtherSpheres.size() << std::endl;
	}

}
