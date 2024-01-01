
find_path( GLAD_INCLUDE_DIR 
    NAMES
        glad/glad.h 
    HINTS
        "${Glad_LOCATION}/include"
        "$ENV{Glad_LOCATION}/include"
    PATHS
		D:/ProgrammingProjects/glad/install/include
    DOC 
        "The directory where Glad headers resides"
)


find_file( GLAD_LIBRARY
	NAMES 
		GLAD.lib
    HINTS
        "${Glad_LOCATION}/install/lib"
        "$ENV{Glad_LOCATION}/install/lib"
	PATHS
		D:/ProgrammingProjects/glad/install/lib
	DOC 
		"The Glad library"
)

set( GLAD_FOUND "NO" )

if(GLAD_INCLUDE_DIR)

    if(GLAD_LIBRARY)
        set (GLAD_FOUND "YES" )
        set (GLAD_INCLUDE_PATH "${Glad_INCLUDE_DIR}")
        set (GLAD_LIB "${Glad_LIB}")
    endif(GLAD_LIBRARY)

endif(GLAD_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(GLAD 
    REQUIRED_VARS
        GLAD_INCLUDE_DIR
        GLAD_LIBRARY
)


