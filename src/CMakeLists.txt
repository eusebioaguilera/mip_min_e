cmake_minimum_required(VERSION 2.8)

# Add ./cmake to CMake modules
list (APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(WITH_GLPK ON CACHE BOOL "Uses the GLPK LP/MIP solver")
set(WITH_GUROBI OFF CACHE BOOL "Uses the GUROBI LP/MIP solver")

FIND_PACKAGE( OpenCV 2.4 REQUIRED )

if (WITH_GUROBI)
	find_package(GUROBI REQUIRED)
	add_definitions(-DUSE_GUROBI=1)
	include_directories(${GUROBI_INCLUDE_DIR})
endif (WITH_GUROBI)

if (WITH_GLPK)
	find_package(GLPK REQUIRED)
	include_directories(${GLPK_INCLUDE_DIR})
endif (WITH_GLPK)


set(LIB_SRC
	contour.cpp
	rect.cpp
	tools.cpp
	contextalgorithm.cpp
)


if (WITH_GLPK)
	set(LIB_SRC ${LIB_SRC} glpkstrategyalgorithm.cpp)
endif (WITH_GLPK)

if (WITH_GUROBI)
	set(LIB_SRC ${LIB_SRC} gurobistrategyalgorithm.cpp)
endif (WITH_GUROBI)


set(MIP_MIN_E_SRC_GLPK
	mip_min_e_glpk.cpp
)

set (MIP_MIN_E_BIN_GLPK
	mip_min_e_glpk
)


set(MIP_MIN_E_SRC_GUROBI
	mip_min_e_gurobi.cpp
)

set (MIP_MIN_E_BIN_GUROBI
	mip_min_e_gurobi
)

add_library(contour STATIC ${LIB_SRC})

if (WITH_GLPK)
	target_link_libraries(contour ${GLPK_LIBRARIES})
endif (WITH_GLPK)

if (WITH_GUROBI)
	target_link_libraries(contour ${GUROBI_LIBRARIES})
endif (WITH_GUROBI)

target_link_libraries(contour ${OpenCV_LIBS})

include_directories("${PROJECT_SOURCE_DIR}")

#Sends the -Wall flag to the gcc compiler
add_definitions(-Wall)

# Set Flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

if (WITH_GLPK)
add_executable(${MIP_MIN_E_BIN_GLPK} ${MIP_MIN_E_SRC_GLPK})
target_link_libraries(${MIP_MIN_E_BIN_GLPK} contour)
endif (WITH_GLPK)

if (WITH_GUROBI)
add_executable(${MIP_MIN_E_BIN_GUROBI} ${MIP_MIN_E_SRC_GUROBI})
target_link_libraries(${MIP_MIN_E_BIN_GUROBI} contour)
endif (WITH_GUROBI)

