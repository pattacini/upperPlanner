# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Nguyen Dong Hai Phuong
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(reaching-supervisor-thread)
aux_source_directory(. SRC_LIST)

set (incldue_files	${CMAKE_CURRENT_SOURCE_DIR}/reachingSupervisorThread.h)
#		 	${CMAKE_CURRENT_SOURCE_DIR}/include/upperPlanner.h)

set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/reachingSupervisorThread.cpp
		 	${CMAKE_CURRENT_SOURCE_DIR}/supervisorThreadModule.cpp)
#		 	${CMAKE_CURRENT_SOURCE_DIR}/src/plannerModule.cpp)

include_directories(	${CMAKE_CURRENT_SOURCE_DIR})
#		    	${CMAKE_CURRENT_SOURCE_DIR}/lib/plan
#		  	${CMAKE_CURRENT_SOURCE_DIR}/lib/rrt-star
#                   	${YARP_INCLUDE_DIRS}
#                    	${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECTNAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} )

install (TARGETS ${PROJECTNAME} DESTINATION bin)

