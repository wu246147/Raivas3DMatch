# 递归解析实际使用的 PCL/VTK DLL，只复制非系统依赖，避免部署整套 SDK。
if(NOT EXISTS "${APP}" OR NOT EXISTS "${DUMPBIN}")
    message(FATAL_ERROR "点云运行时部署缺少程序或 dumpbin：${APP} / ${DUMPBIN}")
endif()
set(CMAKE_GET_RUNTIME_DEPENDENCIES_PLATFORM "windows+pe")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_TOOL "dumpbin")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND "${DUMPBIN}")
get_filename_component(_destination "${APP}" DIRECTORY)
file(GET_RUNTIME_DEPENDENCIES EXECUTABLES "${APP}"
    DIRECTORIES "${_destination}" "${PCL_BIN}" "${VTK_BIN}" "${QT_BIN}" "${BOOST_BIN}"
        "${PCL_BIN}/../3rdParty/OpenNI2/Redist" "C:/Program Files/OpenNI2/Redist"
    RESOLVED_DEPENDENCIES_VAR _resolved UNRESOLVED_DEPENDENCIES_VAR _unresolved
    PRE_EXCLUDE_REGEXES "api-ms-.*" "ext-ms-.*"
    POST_EXCLUDE_REGEXES ".*[/\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\].*")
if(_unresolved)
    message(FATAL_ERROR "无法找到点云视图运行依赖：${_unresolved}")
endif()
foreach(_dependency IN LISTS _resolved)
    get_filename_component(_source_dir "${_dependency}" DIRECTORY)
    if(NOT _source_dir STREQUAL _destination)
        file(COPY "${_dependency}" DESTINATION "${_destination}")
    endif()
endforeach()
