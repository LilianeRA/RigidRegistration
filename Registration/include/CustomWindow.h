#ifndef CUSTOMWINDOW_H
#define CUSTOMWINDOW_H

#include "WindowGLFW.h"

class CustomWindow : public WindowGLFW
{
	public:
		CustomWindow(bool bidimensional, const std::string& title, int width = 1080, int height = 720);
		virtual ~CustomWindow();

		void AppendDrawableSphere(DrawableSpheres* ds);

	protected:
		virtual void SetCustomWindow() override;
		virtual void CustomDraw() override;
		virtual void CustomShutdown() override;

	private:

		std::vector<DrawableSpheres*> mOtherSpheres;
};
#endif // CUSTOMWINDOW_H
