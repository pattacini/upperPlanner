# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Nguyen Dong Hai Phuong
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(reaching-supervisor)
#cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set (incldue_files	${CMAKE_CURRENT_SOURCE_DIR}/reachingSupervisor.h)
#		 	${CMAKE_CURRENT_SOURCE_DIR}/include/upperPlanner.h)

set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/reachingSupervisor.cpp
		 	${CMAKE_CURRENT_SOURCE_DIR}/supervisorModule.cpp)
#		 	${CMAKE_CURRENT_SOURCE_DIR}/src/plannerModule.cpp)

include_directories(	${CMAKE_CURRENT_SOURCE_DIR})
#		    	${CMAKE_CURRENT_SOURCE_DIR}/lib/plan
#		  	${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star
#                   	${YARP_INCLUDE_DIRS}
#                    	${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files}) 
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} )

install (TARGETS ${PROJECT_NAME} DESTINATION bin)
