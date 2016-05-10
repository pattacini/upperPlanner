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



set(lib_files 	${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/kdtree.c
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/kdtree.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/rrts.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/rrts.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/system_single_integrator.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star/system_single_integrator.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/lib/plan/motionPlan.h
		${CMAKE_CURRENT_SOURCE_DIR}/lib/plan/motionPlan.cpp)

#set (incldue_files	${CMAKE_CURRENT_SOURCE_DIR}/include/singlePlanner.h
#		 	${CMAKE_CURRENT_SOURCE_DIR}/include/upperPlanner.h)

#set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/src/singlePlanner.cpp
#		 	${CMAKE_CURRENT_SOURCE_DIR}/src/upperPlanner.cpp
#		 	${CMAKE_CURRENT_SOURCE_DIR}/src/plannerModule.cpp)

include_directories(	${CMAKE_CURRENT_SOURCE_DIR}/lib/plan
		  	${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star
                   	${YARP_INCLUDE_DIRS}
                    	${ICUB_INCLUDE_DIRS})

#icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
#                                           EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} 
#                                           DESTINATION include/iCub/${PROJECT_NAME}
#                                           FILES ${folder_header})

add_subdirectory(modules)

#add_subdirectory(app)

icubcontrib_add_uninstall_target()

#add_definitions(-D_USE_MATH_DEFINES)

#add_executable(${PROJECT_NAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files}) 
#target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})

#install (TARGETS ${PROJECT_NAME} DESTINATION bin)

