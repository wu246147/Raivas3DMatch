# Qt Creator 切换 Kit 或配置会新建独立缓存；在启用后端且路径为空时按本机源码布局补齐 SDK。
# 环境变量提供的候选优先于仓库相对路径；非空的显式缓存值继续由调用方管理。
if(WIN32 AND SM_ENABLE_RAIVAS)
    if(NOT RAIVAS_ROOT)
        find_path(_local_raivas_root raivasalgobbdet.h
            HINTS "$ENV{RAIVAS_ROOT}" "${CMAKE_CURRENT_SOURCE_DIR}/../../OCR/ocr_alg_yolo_obb_opencv"
            NO_DEFAULT_PATH)
        if(_local_raivas_root)
            set(RAIVAS_ROOT "${_local_raivas_root}" CACHE PATH "ocr_alg_yolo_obb_opencv 源码目录" FORCE)
        endif()
    endif()
    if(NOT RAIVAS_RUNTIME_DIR AND RAIVAS_ROOT)
        find_file(_local_raivas_dll RaiVasAlgOBBDet.dll
            HINTS "$ENV{RAIVAS_RUNTIME_DIR}" "${RAIVAS_ROOT}/build/codex-openvino-check/Release"
                "${RAIVAS_ROOT}/build/Desktop_Qt_5_15_2_MSVC2019_64bit-Release" "${RAIVAS_ROOT}/build/Release"
            NO_DEFAULT_PATH)
        if(_local_raivas_dll)
            get_filename_component(_local_raivas_runtime "${_local_raivas_dll}" DIRECTORY)
            set(RAIVAS_RUNTIME_DIR "${_local_raivas_runtime}" CACHE PATH "RaiVas DLL 及其依赖所在目录" FORCE)
        endif()
    endif()
endif()

if(WIN32 AND SM_ENABLE_DEPTH_FILTER)
    if(NOT DEPTH_FILTER_ROOT)
        find_path(_local_depth_filter_root DepthRadiusFilter.h
            HINTS "$ENV{DEPTH_FILTER_ROOT}" "${CMAKE_CURRENT_SOURCE_DIR}/../../../study/qt/pcl_rescale_ply"
            NO_DEFAULT_PATH)
        if(_local_depth_filter_root)
            set(DEPTH_FILTER_ROOT "${_local_depth_filter_root}" CACHE PATH "pcl_rescale_ply 原始头文件目录" FORCE)
        endif()
    endif()
    if(NOT DEPTH_FILTER_RUNTIME_DIR AND DEPTH_FILTER_ROOT)
        find_file(_local_depth_filter_dll RaivasAlgDepthFilter.dll
            HINTS "$ENV{DEPTH_FILTER_RUNTIME_DIR}"
                "${DEPTH_FILTER_ROOT}/build/Desktop_Qt_5_15_2_MSVC2019_64bit-Release"
                "${DEPTH_FILTER_ROOT}/build/Release"
            NO_DEFAULT_PATH)
        if(_local_depth_filter_dll)
            get_filename_component(_local_depth_filter_runtime "${_local_depth_filter_dll}" DIRECTORY)
            set(DEPTH_FILTER_RUNTIME_DIR "${_local_depth_filter_runtime}" CACHE PATH "RaivasAlgDepthFilter DLL 及依赖所在目录" FORCE)
        endif()
    endif()
endif()
