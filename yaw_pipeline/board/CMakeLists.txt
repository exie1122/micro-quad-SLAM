cmake_minimum_required(VERSION 3.15)

set(CMAKE_C_COMPILER "$ENV{COMPILER}/riscv64-unknown-linux-musl-gcc")
set(CMAKE_CXX_COMPILER "$ENV{COMPILER}/riscv64-unknown-linux-musl-g++")
project(yaw_live C CXX)
set(CMAKE_CXX_STANDARD 11)

set(COMMON_FLAGS "-mcpu=c906fdv -march=rv64imafdcv0p7xthead -mcmodel=medany -mabi=lp64d -O3 -DSENSOR_GCORE_GC4653 -D_MIDDLEWARE_V2_ -DC906 -DCV181X -fsigned-char -Wno-format-truncation")
set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}")

set(NANO_BUILD "$ENV{HOME}/LicheeRV-Nano-Build" CACHE PATH "LicheeRV Nano SDK checkout")
set(CAMERA_PROJECT "$ENV{HOME}/LicheeRVNano/Projects/YoloCamera" CACHE PATH "Known-good CSI middleware project")
set(OPENCV_MOBILE_DIR "${CAMERA_PROJECT}/libs/opencv-mobile-4.10.0-licheerv-nano")
set(VENDOR_MW_DIR "${NANO_BUILD}/middleware/v2")
set(VENDOR_OSDRV_DIR "${NANO_BUILD}/osdrv/interdrv/v2/include")
set(TDL_SDK_DIR "$ENV{SDK_PATH}" CACHE PATH "CVITEK TDL SDK root")

include_directories(
    ${OPENCV_MOBILE_DIR}/include/opencv4
    ${VENDOR_MW_DIR}/include
    ${VENDOR_MW_DIR}/sample/common
    ${VENDOR_OSDRV_DIR}/common/uapi
    ${VENDOR_OSDRV_DIR}/chip/mars/uapi
    ${TDL_SDK_DIR}/include
    ${TDL_SDK_DIR}/include/cvi_tdl
    ${TDL_SDK_DIR}/sample/utils
    ${TDL_SDK_DIR}/sample/3rd/tpu/include
    ${TDL_SDK_DIR}/sample/3rd/rtsp/include/cvi_rtsp
    ${VENDOR_MW_DIR}/include/isp/sg200x
)

add_executable(yaw_live
    yaw_live.cpp
    ${CAMERA_PROJECT}/camera_middleware.c
    ${VENDOR_MW_DIR}/sample/common/sample_common_bin.c
    ${VENDOR_MW_DIR}/sample/common/sample_common_isp.c
    ${VENDOR_MW_DIR}/sample/common/sample_common_sensor.c
    ${VENDOR_MW_DIR}/sample/common/sample_common_sys.c
    ${VENDOR_MW_DIR}/sample/common/sample_common_venc.c
    ${VENDOR_MW_DIR}/sample/common/sample_common_vi.c
    ${VENDOR_MW_DIR}/sample/common/sample_common_vpss.c
)

target_link_libraries(yaw_live
    -Wl,-Bstatic
    -Wl,--start-group
    ${VENDOR_MW_DIR}/lib/libsns_full.a
    ${VENDOR_MW_DIR}/lib/libisp.a
    ${VENDOR_MW_DIR}/lib/libvdec.a
    ${VENDOR_MW_DIR}/lib/libvenc.a
    ${VENDOR_MW_DIR}/lib/libawb.a
    ${VENDOR_MW_DIR}/lib/libae.a
    ${VENDOR_MW_DIR}/lib/libaf.a
    ${VENDOR_MW_DIR}/lib/libcvi_bin.a
    ${VENDOR_MW_DIR}/lib/libcvi_bin_isp.a
    ${VENDOR_MW_DIR}/lib/libmisc.a
    ${VENDOR_MW_DIR}/lib/libisp_algo.a
    ${VENDOR_MW_DIR}/lib/libsys.a
    ${VENDOR_MW_DIR}/lib/libvpu.a
    ${TDL_SDK_DIR}/sample/3rd/middleware/v2/lib/3rd/libini.a
    ${OPENCV_MOBILE_DIR}/lib/libopencv_imgproc.a
    ${OPENCV_MOBILE_DIR}/lib/libopencv_core.a
    -Wl,--end-group
    -Wl,-Bdynamic
    -L${TDL_SDK_DIR}/sample/3rd/middleware/v2/lib -lcvi_ive
    -L${TDL_SDK_DIR}/sample/3rd/tpu/lib -lcviruntime -lcvikernel -lcvimath -lz
    -lpthread -latomic -ldl -lm
)
