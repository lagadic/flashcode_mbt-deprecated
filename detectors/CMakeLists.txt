set(DETECTORS_BASE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DETECTORS_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/detector_base.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(datamatrix)
add_subdirectory(qrcode)