list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(XCM REQUIRED)
find_package(Xenomai QUIET)
find_package(Eigen3 REQUIRED)

include_directories(
   ${EIGEN3_INCLUDE_DIRS}
   ${XBotInterface_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
   ${xbot_msgs_INCLUDE_DIRS}
)

add_library(AuxRosPublisher src/AuxRosPublisher.cpp)
target_link_libraries(AuxRosPublisher ${XCM_LIBRARIES})

set(ENABLE_XENO CACHE BOOL "ON")
if(ENABLE_XENO AND Xenomai_FOUND)
    set_xeno_flags(AuxRosPublisher)
endif()

install(TARGETS AuxRosPublisher
    DESTINATION lib)

