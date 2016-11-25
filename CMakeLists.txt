# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Nguyen Dong Hai Phuong
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(reaching-with-avoidance)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(YARP)
find_package(ICUB)
find_package(ICUBcontrib)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES})   # add yarp definitions - this contains also -D_REENTRANT
include(YarpInstallationHelpers)   # pick up yarp's cmake scripts

set(main_project_path ${CMAKE_CURRENT_SOURCE_DIR})
set(main_project_name ${PROJECT_NAME})

set(lib_files 	${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/kdtree.c
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/kdtree.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/rrts.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/rrts.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/system_single_integrator.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/system_single_integrator.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/lib/include/iCub/motionPlan/motionPlan.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/src/motionPlan.cpp)

include_directories(	${CMAKE_CURRENT_SOURCE_DIR}/lib/include
			${CMAKE_CURRENT_SOURCE_DIR}/lib/include/iCub/motionPlan
			${CMAKE_CURRENT_SOURCE_DIR}/lib/src
		  	${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star
#		  	${CMAKE_CURRENT_SOURCE_DIR}/lib/particleThread
                   	${YARP_INCLUDE_DIRS}
                    	${ICUB_INCLUDE_DIRS})


add_subdirectory(modules)

add_subdirectory(lib)

add_subdirectory(app)

icubcontrib_add_uninstall_target()

#add_definitions(-D_USE_MATH_DEFINES)

#add_executable(${PROJECT_NAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files}) 
#target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})

#install (TARGETS ${PROJECT_NAME} DESTINATION bin)

