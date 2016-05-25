# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Nguyen Dong Hai Phuong
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(reaching-planner)
aux_source_directory(. SRC_LIST)


set (include_files	${CMAKE_CURRENT_SOURCE_DIR}/include/singlePlanner.h
		 	${CMAKE_CURRENT_SOURCE_DIR}/include/upperPlanner.h)

set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/src/singlePlanner.cpp
		 	${CMAKE_CURRENT_SOURCE_DIR}/src/upperPlanner.cpp
		 	${CMAKE_CURRENT_SOURCE_DIR}/src/plannerModule.cpp)

include_directories(	${CMAKE_CURRENT_SOURCE_DIR}
			${CMAKE_CURRENT_SOURCE_DIR}/include)

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files}) 
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})

install (TARGETS ${PROJECT_NAME} DESTINATION bin)

