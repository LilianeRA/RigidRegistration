find_path(
  GLM_INCLUDE_DIR "glm/glm.hpp"
  /usr/include
  D:/ProgrammingProjects/glm-0.9.9.8)

SET(GLM_FOUND "NO")
IF(GLM_INCLUDE_DIR)
    SET(GLM_FOUND "YES")
ENDIF()
