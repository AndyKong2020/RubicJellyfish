cmake_minimum_required(VERSION 3.10)
project(DigitalRecognition)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
# 目前支持的识别方法：MobileNet, BP, Template
if (NOT RECOGNITION_METHOD)
    set(RECOGNITION_METHOD "BP")
endif()
# 目前支持的后端引擎：openvino, tensorrt, 也可以设置为自动或None
if (NOT NN_BACKEND)
    set(NN_BACKEND "auto")
endif()
set(BENCHMARK off)

include(path.cmake)

if (${NN_BACKEND} MATCHES "auto")
    find_package(OpenVINO)
    find_package(CUDA)
    if(EXISTS "/usr/local/include/rknn_api.h")
        set(NN_BACKEND "rknn")
    elseif(${CUDA_FOUND})
        set(NN_BACKEND "tensorrt")
    elseif(${OpenVINO_FOUND})
        set(NN_BACKEND "openvino")
    else()
        message(FATAL_ERROR "No Support NN Backend Found")
    endif()
endif()

function(define_openvino modelName)
    add_definitions(-DOPENVINO)
    add_definitions(-DMODEL_PATH="${PROJECT_SOURCE_DIR}/model/${modelName}.xml")

    find_package(OpenVINO REQUIRED)

    set(SOURCE_FILES openvino/modelManager.cpp PARENT_SCOPE)
    set(INCLUDE_DIRS ${OpenVINO_INCLUDE_DIR} /opt/intel/openvino/runtime/include PARENT_SCOPE)
    set(LINK_LIBRARIES openvino::runtime PARENT_SCOPE)
endfunction(define_openvino)

function(define_rknn modelName)
    add_definitions(-DRKNN)
    add_definitions(-DMODEL_PATH="${PROJECT_SOURCE_DIR}/model/${modelName}.rknn")

    set(SOURCE_FILES rknn/modelManager.cpp PARENT_SCOPE)
    set(LINK_LIBRARIES rknnrt PARENT_SCOPE)
endfunction(define_rknn)

function(define_tensorrt modelName)
    add_definitions(-DTENSORRT)
    add_definitions(-DMODEL_PATH="${PROJECT_SOURCE_DIR}/model/${modelName}.trtmodel")
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
    find_package(CUDA REQUIRED)

    set(CMAKE_CUDA_ARCHITECTURES native)

    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        message("embed_platform on")
        add_definitions(-DJETSON)
        include_directories(/usr/local/cuda/targets/aarch64-linux/include)
        link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
    else ()
        message("embed_platform off")
        # cuda
        include_directories(${CUDA_INCLUDE_DIRS})
        include_directories(${CUDA_}/include)
        link_directories(${CUDA_}/lib64)

        # tensorrt
        include_directories(${TensorRT_DIR_}/include)
        link_directories(${TensorRT_DIR_}/lib)
    endif ()

    cuda_add_library(cupp STATIC tensorrt/preprocess.cu)
    set(SOURCE_FILES tensorrt/modelManager.cpp tensorrt/inferRequest.cpp PARENT_SCOPE)
    set(LINK_LIBRARIES nvinfer ${CUDA_LIBRARIES} cupp PARENT_SCOPE)
endfunction(define_tensorrt)

if(${RECOGNITION_METHOD} MATCHES "MobileNet")
    message(STATUS "Using RECOGNITION METHOD MobileNet")
    add_definitions(-DMOBILE_NET)
    if(${NN_BACKEND} MATCHES "openvino")
        define_openvino("mobileNet")
    elseif(${NN_BACKEND} MATCHES "tensorrt")
        define_tensorrt("mobileNet")
    elseif(${NN_BACKEND} MATCHES "rknn")
        define_rknn("mobileNet")
    else()
        message(FATAL_ERROR "Not Support NN Backend")
    endif()
elseif(RECOGNITION_METHOD MATCHES "BP")
    message(STATUS "Using RECOGNITION METHOD BP")
    add_definitions(-DBP)
    if(${NN_BACKEND} MATCHES "openvino")
        define_openvino("BP")
    elseif(${NN_BACKEND} MATCHES "tensorrt")
        define_tensorrt("BP")
    elseif(${NN_BACKEND} MATCHES "rknn")
        define_rknn("BP")
    else()
        message(FATAL_ERROR "Not Support NN Backend")
    endif()
elseif(RECOGNITION_METHOD MATCHES "Template")
    message(STATUS "Using RECOGNITION METHOD Template")
    message(WARNING "Template method has unresolved issue. If you want to use, please rollback DR to commit b0f8c5d7")
    add_definitions(-DTEMPLATE)
    add_definitions(-DTEMPLATES_PATH="${PROJECT_SOURCE_DIR}/templateMatch/templates/")
    set(SOURCE_FILES templateMatch/modelManager.cpp)
else()
    message(FATAL_ERROR "Not Define RECOGNITION METHOD")
endif()

find_package(OpenCV REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIR}
        ${INCLUDE_DIRS}
)

add_library(DigitalRecognitionLib STATIC ${SOURCE_FILES} InferResult.cpp)
target_link_libraries(DigitalRecognitionLib ${LINK_LIBRARIES} ${OpenCV_LIBRARIES})

if(BENCHMARK)
    add_executable(benchmark benchmark.cpp)
    target_link_libraries(benchmark DigitalRecognitionLib ${OpenCV_LIBRARIES})
endif()
