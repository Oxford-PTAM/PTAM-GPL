cmake_minimum_required (VERSION 2.8.3)
PROJECT(PTAM)

#Tell compiler where to find required libraries
link_directories(/usr/local/lib/ /usr/lib/x86_64-linux-gnu/)

link_libraries(GL cvd GVars3 opencv_core opencv_highgui blas lapack)

include_directories(/usr/local/include/TooN/ ./Src/library/)

#Get lists of the source files
FILE(GLOB SOURCEFILES ./Src/library/*.cpp ./Src/library/*.c ./Src/library/*.cc)

FILE(GLOB PTAM_EXECUTABLE_SOURCE ./Src/executables/PTAM/*.cpp ./Src/executables/PTAM/*.c ./Src/executables/PTAM/*.cc)

FILE(GLOB PTAM_CAMERA_CALIBRATOR_SOURCE ./Src/executables/PTAM_camera_calibrator/*.cpp ./Src/executables/PTAM_camera_calibrator/*.c ./Src/executables/PTAM_camera_calibrator/*.cc)

#Set the binaries to be placed in the ./bin/ directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin/")

#Create a static library of the common source files so they are not built twice
add_library(PTAM_library STATIC  ${SOURCEFILES})


#Tell CMake what executables to make
ADD_EXECUTABLE(PTAM ${PTAM_EXECUTABLE_SOURCE}) 

ADD_EXECUTABLE(PTAM_camera_calibrator ${PTAM_CAMERA_CALIBRATOR_SOURCE}) 

#link libraries to executable
target_link_libraries(PTAM GL cvd GVars3  PTAM_library opencv_core opencv_highgui)

target_link_libraries(PTAM_camera_calibrator GL cvd GVars3  PTAM_library opencv_core opencv_highgui)

