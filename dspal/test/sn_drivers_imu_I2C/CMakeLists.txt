cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "akushley@qti.qualcomm.com")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/external/cmake_hexagon")
include(qurt_flags)
include(bundle)


include_directories(dspal/include)
include(qurt_flags)
include(sn_drivers.cmake)
