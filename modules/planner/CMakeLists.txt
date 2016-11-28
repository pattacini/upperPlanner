# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Nguyen Dong Hai Phuong
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#project(reaching-planner)

set(PROJECTNAME reaching-planner)
project(${PROJECTNAME})
aux_source_directory(. SRC_LIST)

set (generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")

set (include_files	${CMAKE_CURRENT_SOURCE_DIR}/include/singlePlanner.h
		 	${CMAKE_CURRENT_SOURCE_DIR}/include/upperPlanner.h)

set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/src/singlePlanner.cpp
		 	${CMAKE_CURRENT_SOURCE_DIR}/src/upperPlanner.cpp
		 	${CMAKE_CURRENT_SOURCE_DIR}/src/plannerModule.cpp)

set (msg_files	${CMAKE_CURRENT_SOURCE_DIR}/msg/object.msg
		${CMAKE_CURRENT_SOURCE_DIR}/msg/SharedData_new.msg)

yarp_idl_to_dir(${CMAKE_CURRENT_SOURCE_DIR}/yarp_msg/object.msg 	${generated_libs_dir})
yarp_idl_to_dir(${CMAKE_CURRENT_SOURCE_DIR}/yarp_msg/SharedData_new.msg ${generated_libs_dir})

include_directories(	${generated_libs_dir}/include/
			${YARP_INCLUDE_DIRS}
			${CMAKE_CURRENT_SOURCE_DIR}
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${CMAKE_CURRENT_SOURCE_DIR}/include/yarp_msg
)

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECTNAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})

install (TARGETS ${PROJECTNAME} DESTINATION bin)

