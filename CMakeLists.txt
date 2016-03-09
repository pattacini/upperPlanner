# Copyright: (C) 2010 RobotCub Consortium
# Author: Nguyen Dong Hai Phuong
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(upperPlanner)
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

include(YarpIDL)

set(idl_files    ${PROJECT_NAME}.thrift)

yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)

source_group("IDL Files" FILES ${idl_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} ${SRC_LIST} rrts.hpp ${IDL_GEN_FILES} ${idl_files}) #system_single_integrator.cpp kdtree.c)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})

install (TARGETS ${PROJECT_NAME} DESTINATION bin)

#add_executable(planner main.cpp particlePlannedThread.cpp system_single_integrator.cpp kdtree.c)
#target_link_libraries(planner ${YARP_LIBRARIES} ctrlLib iKin)

#add_executable(plannedReactController plannedReactController.cpp ${header_files} ${source_files} ${IDL_GEN_FILES} ${idl_files} particlePlannedThread.cpp system_single_integrator.cpp kdtree.c)
#target_link_libraries(plannedReactController ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})

#install(TARGETS planner plannedReactController DESTINATION bin)
#install(TARGETS plannedReactController DESTINATION bin)
#install(TARGETS planner DESTINATION bin)


