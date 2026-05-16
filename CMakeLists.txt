cmake_minimum_required(VERSION 3.10)
project(openloong-dyn-control)

IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
ENDIF ()
add_compile_options(-std=c++17)

set(OPENLOONG_HAS_ROS2 FALSE)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(std_msgs QUIET)
if(rclcpp_FOUND AND sensor_msgs_FOUND AND std_msgs_FOUND)
	set(OPENLOONG_HAS_ROS2 TRUE)
	message("ROS2 found: ROS2 interfaces will be compiled; runtime publishing/subscribing is controlled by CLI and JSON config.")
else()
	message("ROS2 not found: building simulation-only binaries.")
endif()

#系统eigen版本可能老旧，使用自供eigen
set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")
set(dirQuill "third_party/quill")

set(libURDF "third_party/urdfdom")

set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
set(incMujoco "third_party/mujoco")
set(incQP "third_party/qpOASES")

set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirJson} ${dirQuill})
set(allLib ${allDir} ${libURDF})
set(allInc ${allDir} ${incFmt} ${incBoost} ${incMujoco} ${incQP})

include_directories(${allInc})
include_directories("algorithm")
include_directories("common")
include_directories("math")
include_directories("sim_interface")

message(${CMAKE_SYSTEM_PROCESSOR})
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	list(APPEND allLib "third_party/mujoco/lin_arm64")
	list(APPEND allLib "third_party/qpOASES/lin_arm64")
else()
	list(APPEND allLib "third_party/mujoco/lin_x64")
	list(APPEND allLib "third_party/qpOASES/lin_x64")
endif()
link_directories(${allLib})

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp algorithm/*.cpp common/*.cpp math/*.cpp sim_interface/*.cpp)
file(GLOB HEADER_FILES *.h algorithm/*.h common/*.h math/*.h sim_interface/*.h)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})

#链接库
set(corLibs pinocchio urdfdom_model tinyxml console_bridge jsoncpp quill)
set(simLibs glfw3)#mujoco区分x86/arm的方式不一样，在上面按link目录区分

#根据架构，处理连接库
set(sysCoreLibs)
set(sysSimLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysSimLibs dl)#arm 需要 -ldl
else()
	message("linux x64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
	foreach(lib ${simLibs})
		list(APPEND sysSimLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})

#生成控制核心库
add_library(core ${SOURCES})
if(OPENLOONG_HAS_ROS2)
	target_compile_definitions(core PUBLIC OPENLOONG_HAS_ROS2=1)
	target_link_libraries(core
			${sysCoreLibs}
			pthread
			rclcpp::rclcpp
			sensor_msgs::sensor_msgs__rosidl_generator_cpp
		sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
		std_msgs::std_msgs__rosidl_generator_cpp
			std_msgs::std_msgs__rosidl_typesupport_cpp)
else()
	target_compile_definitions(core PUBLIC OPENLOONG_HAS_ROS2=0)
	target_link_libraries(core ${sysCoreLibs} pthread)
endif()

#生成仿真可执行文件
add_executable(walk_wbc_joystick demo/walk_wbc_joystick.cpp)
target_link_libraries(walk_wbc_joystick core mujoco qpOASES ${sysSimLibs} dl)

add_executable(walk_mpc_wbc_joystick demo/walk_mpc_wbc_joystick.cpp)
target_link_libraries(walk_mpc_wbc_joystick core mujoco qpOASES ${sysSimLibs} dl)

add_executable(walk_mpc_wbc_v4 demo/walk_mpc_wbc_v4.cpp)
target_link_libraries(walk_mpc_wbc_v4 core mujoco qpOASES ${sysSimLibs} dl)

add_executable(walk_mpc_wbc_leg demo/walk_mpc_wbc_leg.cpp)
target_link_libraries(walk_mpc_wbc_leg core mujoco qpOASES ${sysSimLibs} dl)
