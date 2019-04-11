include(qurt_lib)

set(CMAKE_VERBOSE_MAKEFILE "ON")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -std=gnu99 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wuninitialized -Wextra -Werror -Wno-cast-align -Wno-strict-aliasing -Wno-unused-parameter")

set(CMAKE_CXX_FLAGS "${CMAKE_CPP_FLAGS} -O3 -std=c++11 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wuninitialized -Wextra -Werror -Wno-cast-align -Wno-strict-aliasing -Wno-sign-compare -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable")
# This is a better set of flags to use:
# set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3 -std=c++11 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wuninitialized -Wextra -Werror -Wno-cast-align -Wno-strict-aliasing -Wno-unused-parameter -Wno-error-sign-compare -Wno-error-unused-function")

include_directories(
  include
  ../../include
  external/dspal/include
  external/gnss
  )

set(snav_q6_srcs
  src/dummy.cpp
  )

link_directories(../external/gnss)
  
add_library(snav_q6 SHARED ${snav_q6_srcs})
set_property(TARGET snav_q6 PROPERTY POSITION_INDEPENDENT_CODE TRUE)
  
#mag0 hmc5883l driver
set(sonar0_maxbotix_driver_srcs
  src/sonar0_maxbotix_driver.cpp
  src/snav_drivers_io.c

  )

add_library(sonar0_maxbotix_driver SHARED ${sonar0_maxbotix_driver_srcs})
set_property(TARGET sonar0_maxbotix_driver PROPERTY POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries( sonar0_maxbotix_driver snav_q6 )

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
