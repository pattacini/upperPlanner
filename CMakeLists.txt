# Copyright: (C) 2010 RobotCub Consortium
# Author: Nguyen Dong Hai Phuong
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(reaching-planner)
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


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} ${SRC_LIST} rrts.hpp) 
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})

install (TARGETS ${PROJECT_NAME} DESTINATION bin)

