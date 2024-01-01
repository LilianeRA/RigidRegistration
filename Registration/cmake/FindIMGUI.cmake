find_path(
  IMGUI_INCLUDE_DIR "imgui.h"
  /usr/include
  D:/ProgrammingProjects/imgui-1.89.4)

find_library(IMGUI_LIBRARY
  NAMES imgui
  PATHS
  "D:/ProgrammingProjects/imgui-1.89.4/install/lib")
 
 
 
set( IMGUI_FOUND "NO" )

if(IMGUI_INCLUDE_DIR)

    if(IMGUI_LIBRARY)
        set (IMGUI_FOUND "YES" )
        set (IMGUI_INCLUDE_PATH "${Visualizer3D_INCLUDE_DIR}")
        set (IMGUI_LIB "${Visualizer3D_LIB}")
    endif(IMGUI_LIBRARY)

endif(IMGUI_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(IMGUI 
    REQUIRED_VARS
        IMGUI_INCLUDE_DIR
        IMGUI_LIBRARY
)