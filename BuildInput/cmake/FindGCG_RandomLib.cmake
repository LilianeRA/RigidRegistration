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

find_path( GCG_RandomLib_INCLUDE_DIR 
    NAMES
        Random.h RandomVector.h RandomGaussian.h 
    HINTS
        "${GCG_RandomLib_LOCATION}/include"
        "$ENV{GCG_RandomLib_LOCATION}/include"
    PATHS
		E:/GIT/GCG_RandomLib/install/include
    DOC 
        "The directory where GCG_RandomLib headers resides"
)


find_file( GCG_RandomLib_LIBRARY
	NAMES 
		GCG_RandomLib_s.lib
    HINTS
        "${GCG_RandomLib_LOCATION}/install/lib"
        "$ENV{GCG_RandomLib_LOCATION}/install/lib"
	PATHS
		E:/GIT/GCG_RandomLib/install/lib
	DOC 
		"The GCG_RandomLib library"
)

set( GCG_RandomLib_FOUND "NO" )

if(GCG_RandomLib_INCLUDE_DIR)

    if(GCG_RandomLib_LIBRARY)
        set (GCG_RandomLib_FOUND "YES" )
        set (GCG_RandomLib_INCLUDE_PATH "${GCG_RandomLib_INCLUDE_DIR}")
        set (GCG_RandomLib_LIB "${GCG_RandomLib_LIB}")
    endif(GCG_RandomLib_LIBRARY)

endif(GCG_RandomLib_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(GCG_RandomLib 
    REQUIRED_VARS
        GCG_RandomLib_INCLUDE_DIR
        GCG_RandomLib_LIBRARY
)

#mark_as_advanced(
#  GCG_RandomLib_INCLUDE_DIR
#  GCG_RandomLib_LIBRARY
#)

