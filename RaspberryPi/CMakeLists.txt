cmake_minimum_required(VERSION 3.11) # FetchContent is available in 3.11+
include(FetchContent)
include(armhf-toolchain.cmake)
set(CMAKE_BUILD_TYPE Debug)
# raspi libraries for threading & gpu
# include_directories( ${CMAKE_SYSROOT}/opt/vc/include ${CMAKE_SYSROOT}/opt/vc/include/interface/vmcs_host/linux ${CMAKE_SYSROOT}/opt/vc/include/interface/vcos/pthreads ${CMAKE_SYSROOT}/opt/vc/include/interface/vcos/generic)
# link_directories(${CMAKE_SYSROOT}/opt/vc/lib)

# ========================== Build libwebsockets ========================================

project(lws)

# LWS config
set(OPENSSL_ROOT_DIR ${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf/)
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
set(CMAKE_TOOLCHAIN_FILE armhf-toolchain.cmake)
set(LWS_WITH_TLS ON)

 # build a debug symbol table for gdb
set(CMAKE_BUILD_TYPE Debug)

# fetch & build libwebsockets
FetchContent_Declare(
    libwebsockets
    GIT_REPOSITORY https://github.com/warmcat/libwebsockets.git
    GIT_TAG v4.0.10
    )
  FetchContent_GetProperties(libwebsockets)
  if(NOT libwebsockets_POPULATED)
    set(FETCHCONTENT_QUIET NO)
    FetchContent_Populate(libwebsockets)
    add_subdirectory(${libwebsockets_SOURCE_DIR} ${libwebsockets_BINARY_DIR})
  endif()
  
  # libwebsockets build dependencies
  include_directories(build/_deps/libwebsockets-build/include ${CMAKE_SYSROOT}/usr/include/arm-linux-gnueabihf)
  link_directories(build/_deps/libwebsockets-build/lib ${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf/openssl-1.0.2)

# =======================================================================================

# ================================ Helper Library ========================================

project(robolib_build)

set( arducam_dir /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/ )

set( robolib_src  
    src/ws/message_queue.c 
    ${arducam_dir}EZCAM.c 
    ${arducam_dir}ArduCAM.c 
    ${arducam_dir}bcm283x_board_driver.c 
    ${arducam_dir}sccb_bus.c
)

# link_directories(build/_deps/raylib-build/src)

add_library(robolib STATIC ${robolib_src} )
# target_link_libraries( ${PROJECT_NAME} rt )

# ========================================================================================

macro(OCV_Build project_name src_file)
    project( ${project_name} ) 
    
    # build a debug symbol table for gdb
    set(CMAKE_BUILD_TYPE Debug)

    add_executable(${PROJECT_NAME}  ${src_file} )

    find_package(OpenCV REQUIRED)
    include_directories(SYSTEM /home/abobco/RaspberryPi/pigpio-master/install/include )
    include_directories(${CMAKE_SYSROOT}/opt/vc/include ${CMAKE_SYSROOT}/opt/vc/include/interface/vmcs_host/linux)
    include_directories(${OpenCV_INCLUDE_DIRS} /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src )
    # link_directories( /home/abobco/RaspberryPi/pigpio-master/install/lib/)
    find_library(PIG NAMES pigpio PATHS /home/abobco/RaspberryPi/pigpio-master/install/lib/ NO_DEFAULT_PATH )
    find_library(PIGD NAMES pigpiod_if2 PATHS /home/abobco/RaspberryPi/pigpio-master/install/lib/ NO_DEFAULT_PATH )

    link_directories(${CMAKE_SYSROOT}/opt/vc/lib/)

    find_library(l_EGL NAMES EGL PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_GLES NAMES brcmGLESv2 PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_bcm_host NAMES bcm_host PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_vcos NAMES vcos PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_vchiq_arm NAMES vchiq_arm PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    target_link_libraries( ${PROJECT_NAME}  ${OpenCV_LIBS} ${PIG} ${PIGD} pthread rt robolib websockets dl ${l_vcos} ${l_EGL} ${l_GLES} ${l_bcm_host} ${l_vchiq_arm})

    # include_directories(build/_deps/libwebsockets-build/include ${raylib_BINARY_DIR}/src ${CMAKE_SYSROOT}/usr/include/lua5.1) # default raspi lua build
    # include_directories(build/_deps/libwebsockets-build/include ${raylib_BINARY_DIR}/src ${raylib_BINARY_DIR}/src/external build/_deps/lua-5.4.0/install/include) # compound assignment lua5.4 build
    # find_library( LUA54 NAMES "lua" PATHS ${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/lua-5.4.0/install/lib NO_DEFAULT_PATH)

    # cross compiler stuff
    set_target_properties(${PROJECT_NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "./")
#  target_link_libraries(${PROJECT_NAME} raylib vcos brcmGLESv2 brcmEGL pthread rt m bcm_host dl websockets ${LUA54} xnlib-fn ${prof} z)
endmacro()

OCV_Build( test src/ocv_version.cpp )
OCV_Build( mat_test src/ocv_mat.cpp )
OCV_Build( threshold src/ocv_threshold.cpp )
OCV_Build( corner src/ocv_harris_corner.cpp )
OCV_Build( face src/ocv_face_finder.cpp )

OCV_Build( servo src/servo_demo.cpp )
OCV_Build( servo_claw src/servo_claw.cpp )
OCV_Build( servo_smooth src/servo_smooth.cpp )
OCV_Build( servo_smooth_array src/servo_smooth_array.cpp )
OCV_Build( client src/client.cpp )
OCV_Build( arducam src/arducam.cpp )
OCV_Build( arducam_lws src/arducam_lws.cpp )
OCV_Build( arm_lws src/arm_lws.cpp )
OCV_Build( servo360 src/servo360.cpp )
OCV_Build( stepper src/stepper.cpp )
OCV_Build( red src/rot_enc.cpp )
OCV_Build( wavetimer src/wavetimer.cpp )

OCV_Build( xn_test_phys src/XNlib/xn_test_phys.cpp )

# simulation build
OCV_Build( egl_info src/XNlib/egl/egl_info.cpp )
OCV_Build( egl_test src/XNlib/egl/egl_tests.cpp )
OCV_Build( egl_tri src/XNlib/opengl/egl_tri.cpp )
