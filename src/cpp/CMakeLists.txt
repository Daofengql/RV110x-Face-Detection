cmake_minimum_required(VERSION 3.10)

# 将原先的 rknn_yolov5_demo 改为 AI_Eye_Tracker
project(AI_Face_Tracker)
add_compile_options(-O3)

# 默认使用 rknpu2/yolov5.cc
set(rknpu_yolov5_file rknpu2/yolov5.cc)




if (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
    add_definitions(-DRV1106_1103)
    set(rknpu_yolov5_file rknpu2/yolov5_rv1106_1103.cc)
    # 包含 dma allocator
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/allocator/dma)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/opencv/rv110x)
    #include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty)

elseif(TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    add_definitions(-DRKNPU1)
    set(rknpu_yolov5_file rknpu1/yolov5.cc)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty)
# 与 3rdparty、utils 同级，都在 src 下
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../utils     utils.out)

set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )


# 获取当前目录下所有 .cc 源文件
file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    ${rknpu_yolov5_file}
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    imageutils
    fileutils
    imagedrawing
    ${LIBRKNNRT}
    dl
)

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME} log)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} Threads::Threads)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBRKNNRT_INCLUDES}
)

# 安装可执行文件到安装前缀根目录下
install(TARGETS ${PROJECT_NAME} DESTINATION .)

# 安装模型文件到 ./model 下
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../model/list.txt
    DESTINATION ./model
)

file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION ./model)

