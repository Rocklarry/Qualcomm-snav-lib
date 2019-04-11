set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_hexagon")
#include(qurt_flags)
#include(bundle)

include_directories(
	../../include
	../include
	)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-gnu-array-member-paren-init -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing") 


include_directories(
	snav_drivers.h
	snav_drivers_io.h
)
#mag0 hmc5883l driver 
set(mag0_hmc5883l_driver_srcs
mag0_hmc5883l_driver.cpp
snav_drivers_io.c 
) 

link_directories("/home/idea/work/dspal/dspal/test/mag/lib")
add_library(mag0_hmc5883l_driver SHARED ${mag0_hmc5883l_driver_srcs}) 
set_property(TARGET mag0_hmc5883l_driver PROPERTY POSITION_INDEPENDENT_CODE TRUE) 
target_link_libraries( mag0_hmc5883l_driver snav_q6 ) 

