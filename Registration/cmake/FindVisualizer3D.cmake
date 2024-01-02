#
#   Copyright 2013 Pixar
#
#   Licensed under the Apache License, Version 2.0 (the "Apache License")
#   with the following modification; you may not use this file except in
#   compliance with the Apache License and the following modification to it:
#   Section 6. Trademarks. is deleted and replaced with:
#
#   6. Trademarks. This License does not grant permission to use the trade
#      names, trademarks, service marks, or product names of the Licensor
#      and its affiliates, except as required to comply with Section 4(c) of
#      the License and to reproduce the content of the NOTICE file.
#
#   You may obtain a copy of the Apache License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the Apache License with the above modification is
#   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#   KIND, either express or implied. See the Apache License for the specific
#   language governing permissions and limitations under the Apache License.
#

# Try to find GLFW library and include path.
# Once done this will define
#
# GLFW_FOUND
# GLFW_INCLUDE_DIR
# GLFW_LIBRARIES
#

find_path( Visualizer3D_INCLUDE_DIR 
    NAMES
        WindowGLFW.h
    HINTS
        "${Visualizer3D_LOCATION}/include"
        "$ENV{Visualizer3D_LOCATION}/include"
    PATHS
		E:/GIT/Visualizer3D/install/include
    DOC 
        "The directory where Visualizer3D headers resides"
)


find_file( Visualizer3D_LIBRARY
	NAMES 
		Visualizer3D_s.lib
    HINTS
        "${Visualizer3D_LOCATION}/install/lib"
        "$ENV{Visualizer3D_LOCATION}/install/lib"
	PATHS
		E:/GIT/Visualizer3D/install/lib
	DOC 
		"The Visualizer3D library"
)


set( Visualizer3D_FOUND "NO" )

if(Visualizer3D_INCLUDE_DIR)

    if(Visualizer3D_LIBRARY)
        set (Visualizer3D_FOUND "YES" )
        set (Visualizer3D_INCLUDE_PATH "${Visualizer3D_INCLUDE_DIR}")
        set (Visualizer3D_LIB "${Visualizer3D_LIB}")
        #set (Visualizer3D_DEPENDENCIES "${LIBRARIES}")
    endif(Visualizer3D_LIBRARY)

endif(Visualizer3D_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Visualizer3D 
    REQUIRED_VARS
        Visualizer3D_INCLUDE_DIR
        Visualizer3D_LIBRARY
		#Visualizer3D_DEPENDENCIES
)

#mark_as_advanced(
#  Visualizer3D_INCLUDE_DIR
#  Visualizer3D_LIBRARY
#)

