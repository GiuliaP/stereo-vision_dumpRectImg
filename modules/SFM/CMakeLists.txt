# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Giulia Pasquale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME SFM_dumpRectImg)
project(${PROJECTNAME})

if(USE_SIFT_GPU)
    find_package(SIFTGPU)
    if(SIFTGPU_FOUND)
        include_directories(${SIFTGPU_INCLUDE_DIRS})
    endif()
endif()

if(USE_SIFT_GPU)
	add_definitions(-DUSING_GPU)
endif()

set(source SFM.cpp)
set(header SFM.h )

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(${PROJECTNAME} ${source} ${header})

target_link_libraries(${PROJECTNAME} stereoVision_dumpRectImg iKin ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

if(USE_SIFT_GPU)
	target_link_libraries(${PROJECTNAME} ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS ${PROJECTNAME} DESTINATION bin)
