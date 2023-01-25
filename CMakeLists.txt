cmake_minimum_required(VERSION 3.3)
project(artec_scanner_robotraconteur_driver)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteurCompanion REQUIRED)
find_package(ArtecSDK REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS
    experimental.artec_scanner.robdef
	MASTER_HEADER
	INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/robdef ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
	AUTO_IMPORT
	)

add_executable(artec_scanner_robotraconteur_driver
    src/artec_scanner_robotraconteur_driver.cpp
	src/artec_scanner_impl.cpp
	src/artec_scanner_util.cpp
	src/artec_scanning_procedure.cpp
    ${RR_THUNK_HDRS}
	${RR_THUNK_SRCS}
)

target_include_directories(artec_scanner_robotraconteur_driver PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(artec_scanner_robotraconteur_driver RobotRaconteurCompanion RobotRaconteurCore 
ArtecSDK::Base ArtecSDK::Algorithms ArtecSDK::Capturing ArtecSDK::Scanning ArtecSDK::Project Eigen3::Eigen)

install(TARGETS artec_scanner_robotraconteur_driver)
