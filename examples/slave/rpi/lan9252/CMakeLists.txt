set(TARGET_NAME easycat_raspberrypi)
add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/od_populator.cc)
target_link_libraries(${TARGET_NAME} kickcat)
set_kickcat_properties(${TARGET_NAME})

find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    set(CAM_TARGET eoe_camera_slave_rpi)
    add_executable(${CAM_TARGET}
        ${CMAKE_CURRENT_SOURCE_DIR}/main_eoe_camera.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/od_populator.cc)
    target_link_libraries(${CAM_TARGET} kickcat opencv::opencv)
    set_kickcat_properties(${CAM_TARGET})
endif()
