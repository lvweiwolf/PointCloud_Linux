# =============================================================================
# ThirdPart CMake Toolchain File
# 第三方库工具链文件 (无CMake Config版本)
# 
# 使用方法:
#   cmake -DCMAKE_TOOLCHAIN_FILE=/home/lvwei/cloud/ThirdPart/toolchain.cmake ..
# =============================================================================

# -----------------------------------------------------------------------------
# 基本路径设置
# -----------------------------------------------------------------------------
# set(THIRDPART_ROOT "")

set(THIRDPART_INCLUDE_DIR "${THIRDPART_ROOT}/include")
set(THIRDPART_LIB_DIR "${THIRDPART_ROOT}/lib")
set(THIRDPART_LIB64_DIR "${THIRDPART_ROOT}/lib64")
set(THIRDPART_BIN_DIR "${THIRDPART_ROOT}/bin")

include_directories(SYSTEM ${THIRDPART_INCLUDE_DIR})
link_directories(${THIRDPART_LIB_DIR} ${THIRDPART_LIB64_DIR})

# =============================================================================
# OpenCV
# =============================================================================
set(OpenCV_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/opencv2")
set(OpenCV_LIBS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
    opencv_videoio
    opencv_video
    opencv_calib3d
    opencv_features2d
    opencv_flann
    opencv_dnn
    opencv_ml
    opencv_objdetect
    opencv_photo
    opencv_shape
    opencv_stitching
    opencv_superres
    opencv_videostab
)
set(OpenCV_FOUND TRUE)

macro(target_link_opencv target)
    target_include_directories(${target} PRIVATE ${OpenCV_INCLUDE_DIR})
    foreach(_lib ${OpenCV_LIBS})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# PCL (Point Cloud Library)
# =============================================================================
set(PCL_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/pcl-1.8")
set(PCL_LIBS
    pcl_common
    pcl_features
    pcl_filters
    pcl_io
    pcl_io_ply
    pcl_kdtree
    pcl_keypoints
    pcl_ml
    pcl_octree
    pcl_recognition
    pcl_registration
    pcl_sample_consensus
    pcl_search
    pcl_segmentation
    pcl_stereo
    pcl_surface
    pcl_tracking
)
set(PCL_FOUND TRUE)

macro(target_link_pcl target)
    target_include_directories(${target} PRIVATE ${PCL_INCLUDE_DIR})
    foreach(_lib ${PCL_LIBS})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# Boost
# =============================================================================
set(Boost_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/boost")
set(Boost_FOUND TRUE)

set(Boost_LIBRARIES
    boost_system
    boost_filesystem
    boost_thread
    boost_date_time
    boost_regex
    boost_serialization
    boost_program_options
    boost_iostreams
    boost_log
    boost_log_setup
    boost_chrono
    boost_timer
    boost_random
    boost_atomic
    boost_container
    boost_context
    boost_contract
    boost_coroutine
    boost_fiber
    boost_graph
    boost_locale
    boost_math_c99
    boost_math_c99f
    boost_math_tr1
    boost_math_tr1f
    boost_prg_exec_monitor
    boost_signals
    boost_type_erasure
    boost_unit_test_framework
    boost_wave
    boost_wserialization
    boost_exception
    boost_stacktrace_addr2line
    boost_stacktrace_backtrace
    boost_stacktrace_basic
    boost_stacktrace_noop
)

macro(target_link_boost target)
    target_include_directories(${target} PRIVATE ${Boost_INCLUDE_DIR})
    foreach(_lib ${ARGN})
        list(FIND Boost_LIBRARIES boost_${_lib} _idx)
        if(_idx GREATER -1)
            target_link_libraries(${target} boost_${_lib})
        else()
            target_link_libraries(${target} ${_lib})
        endif()
    endforeach()
endmacro()

macro(target_link_boost_all target)
    target_include_directories(${target} PRIVATE ${Boost_INCLUDE_DIR})
    foreach(_lib ${Boost_LIBRARIES})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# OSG (OpenSceneGraph)
# =============================================================================
set(OSG_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(OSG_LIBS
    osg
    osgDB
    osgUtil
    osgGA
    osgViewer
    osgText
    osgTerrain
    osgShadow
    osgFX
    osgSim
    osgParticle
    osgManipulator
    osgVolume
    osgAnimation
    osgPresentation
    osgWidget
    osgUI
    OpenThreads
)
set(OSG_FOUND TRUE)

macro(target_link_osg target)
    target_include_directories(${target} PRIVATE ${OSG_INCLUDE_DIR})
    foreach(_lib ${OSG_LIBS})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# osgEarth
# =============================================================================
set(OSGEARTH_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(OSGEARTH_LIBS
    osgEarth
    osgEarthAnnotation
    osgEarthFeatures
    osgEarthSplat
    osgEarthSymbology
    osgEarthUtil
)
set(OSGEARTH_FOUND TRUE)

macro(target_link_osgearth target)
    target_include_directories(${target} PRIVATE ${OSGEARTH_INCLUDE_DIR})
    foreach(_lib ${OSGEARTH_LIBS})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# GDAL
# =============================================================================
set(GDAL_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(GDAL_LIBRARY gdal)
set(GDAL_FOUND TRUE)

macro(target_link_gdal target)
    target_include_directories(${target} PRIVATE ${GDAL_INCLUDE_DIR})
    target_link_libraries(${target} ${GDAL_LIBRARY})
endmacro()

# =============================================================================
# Eigen3
# =============================================================================
set(Eigen3_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/eigen3")
set(Eigen3_FOUND TRUE)

macro(target_link_eigen3 target)
    target_include_directories(${target} PRIVATE ${Eigen3_INCLUDE_DIR})
endmacro()

# =============================================================================
# Ceres
# =============================================================================
set(Ceres_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(Ceres_LIBRARY ceres)
set(Ceres_FOUND TRUE)

macro(target_link_ceres target)
    target_include_directories(${target} PRIVATE ${Ceres_INCLUDE_DIR})
    target_link_libraries(${target} ${Ceres_LIBRARY})
endmacro()

# =============================================================================
# CGAL
# =============================================================================
set(CGAL_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(CGAL_LIBS CGAL CGAL_Core CGAL_ImageIO)
set(CGAL_FOUND TRUE)

macro(target_link_cgal target)
    target_include_directories(${target} PRIVATE ${CGAL_INCLUDE_DIR})
    foreach(_lib ${CGAL_LIBS})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# Protobuf
# =============================================================================
set(Protobuf_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(Protobuf_LIBRARY protobuf)
set(Protobuf_LITE_LIBRARY protobuf-lite)
set(Protobuf_PROTOC_LIBRARY protoc)
set(Protobuf_FOUND TRUE)

macro(target_link_protobuf target)
    target_include_directories(${target} PRIVATE ${Protobuf_INCLUDE_DIR})
    target_link_libraries(${target} ${Protobuf_LIBRARY})
endmacro()

macro(target_link_protobuf_lite target)
    target_include_directories(${target} PRIVATE ${Protobuf_INCLUDE_DIR})
    target_link_libraries(${target} ${Protobuf_LITE_LIBRARY})
endmacro()

# =============================================================================
# ncnn
# =============================================================================
set(ncnn_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/ncnn")
set(ncnn_LIBRARY ncnn)
set(ncnn_FOUND TRUE)

macro(target_link_ncnn target)
    target_include_directories(${target} PRIVATE ${ncnn_INCLUDE_DIR})
    target_link_libraries(${target} ${ncnn_LIBRARY})
endmacro()

# =============================================================================
# spdlog
# =============================================================================
set(spdlog_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(spdlog_LIBRARY spdlog)
set(spdlog_FOUND TRUE)

macro(target_link_spdlog target)
    target_include_directories(${target} PRIVATE ${spdlog_INCLUDE_DIR})
    target_link_libraries(${target} ${spdlog_LIBRARY})
endmacro()

# =============================================================================
# SQLite3
# =============================================================================
set(SQLite3_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(SQLite3_LIBRARY sqlite3)
set(SQLite3_FOUND TRUE)

macro(target_link_sqlite3 target)
    target_include_directories(${target} PRIVATE ${SQLite3_INCLUDE_DIR})
    target_link_libraries(${target} ${SQLite3_LIBRARY})
endmacro()

# =============================================================================
# curl
# =============================================================================
set(CURL_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/curl")
set(CURL_LIBRARY curl)
set(CURL_FOUND TRUE)

macro(target_link_curl target)
    target_include_directories(${target} PRIVATE ${CURL_INCLUDE_DIR})
    target_link_libraries(${target} ${CURL_LIBRARY})
endmacro()

# =============================================================================
# TIFF
# =============================================================================
set(TIFF_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(TIFF_LIBRARY tiff)
set(TIFF_FOUND TRUE)

macro(target_link_tiff target)
    target_include_directories(${target} PRIVATE ${TIFF_INCLUDE_DIR})
    target_link_libraries(${target} ${TIFF_LIBRARY})
endmacro()

# =============================================================================
# PROJ
# =============================================================================
set(PROJ_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(PROJ_LIBRARY proj)
set(PROJ_FOUND TRUE)

macro(target_link_proj target)
    target_include_directories(${target} PRIVATE ${PROJ_INCLUDE_DIR})
    target_link_libraries(${target} ${PROJ_LIBRARY})
endmacro()

# =============================================================================
# FLANN
# =============================================================================
set(FLANN_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/flann")
set(FLANN_LIBS flann flann_cpp)
set(FLANN_FOUND TRUE)

macro(target_link_flann target)
    target_include_directories(${target} PRIVATE ${FLANN_INCLUDE_DIR})
    foreach(_lib ${FLANN_LIBS})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# TBB
# =============================================================================
set(TBB_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/tbb")
set(TBB_LIBRARY tbb)
set(TBB_FOUND TRUE)

macro(target_link_tbb target)
    target_include_directories(${target} PRIVATE ${TBB_INCLUDE_DIR})
    target_link_libraries(${target} ${TBB_LIBRARY})
endmacro()

# =============================================================================
# pugixml
# =============================================================================
set(pugixml_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}")
set(pugixml_LIBRARY pugixml)
set(pugixml_FOUND TRUE)

macro(target_link_pugixml target)
    target_include_directories(${target} PRIVATE ${pugixml_INCLUDE_DIR})
    target_link_libraries(${target} ${pugixml_LIBRARY})
endmacro()

# =============================================================================
# LASlib
# =============================================================================
link_directories(${THIRDPART_LIB_DIR}/LASlib)
set(LASlib_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/LASlib")
set(LASlib_LIBRARY LASlib)
set(LASlib_FOUND TRUE)

macro(target_link_laslib target)
    target_include_directories(${target} PRIVATE ${LASlib_INCLUDE_DIR})
    target_link_libraries(${target} ${LASlib_LIBRARY})
endmacro()

# =============================================================================
# RapidJSON (header-only)
# =============================================================================
set(RapidJSON_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/rapidjson")
set(RapidJSON_FOUND TRUE)

macro(target_link_rapidjson target)
    target_include_directories(${target} PRIVATE ${RapidJSON_INCLUDE_DIR})
endmacro()

# =============================================================================
# uuid
# =============================================================================
set(UUID_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/uuid")
set(UUID_LIBRARY uuid)
set(UUID_FOUND TRUE)

macro(target_link_uuid target)
    target_include_directories(${target} PRIVATE ${UUID_INCLUDE_DIR})
    target_link_libraries(${target} ${UUID_LIBRARY})
endmacro()

# =============================================================================
# glslang
# =============================================================================
set(glslang_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/glslang")
set(glslang_LIBS
    glslang
    glslang-default-resource-limits
    GenericCodeGen
    MachineIndependent
    OGLCompiler
    OSDependent
    SPIRV
)
set(glslang_FOUND TRUE)

macro(target_link_glslang target)
    target_include_directories(${target} PRIVATE ${glslang_INCLUDE_DIR})
    foreach(_lib ${glslang_LIBS})
        target_link_libraries(${target} ${_lib})
    endforeach()
endmacro()

# =============================================================================
# bit7z
# =============================================================================
set(bit7z_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/bit7z")
set(bit7z_LIBRARY bit7z64)
set(bit7z_FOUND TRUE)

macro(target_link_bit7z target)
    target_include_directories(${target} PRIVATE ${bit7z_INCLUDE_DIR})
    target_link_libraries(${target} ${bit7z_LIBRARY})
endmacro()

# =============================================================================
# VCGLib (header-only)
# =============================================================================
set(VCG_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/vcg")
set(VCG_FOUND TRUE)

macro(target_link_vcg target)
    target_include_directories(${target} PRIVATE ${VCG_INCLUDE_DIR})
endmacro()

# =============================================================================
# img (header-only, stb-style)
# =============================================================================
set(IMG_INCLUDE_DIR "${THIRDPART_INCLUDE_DIR}/img")
set(IMG_FOUND TRUE)

macro(target_link_img target)
    target_include_directories(${target} PRIVATE ${IMG_INCLUDE_DIR})
endmacro()

# =============================================================================
# 通用辅助宏：链接多个库
# =============================================================================
macro(target_link_thirdpart target)
    foreach(_lib ${ARGN})
        string(TOLOWER ${_lib} _lib_lower)
        if(_lib_lower STREQUAL "opencv")
            target_link_opencv(${target})
        elseif(_lib_lower STREQUAL "pcl")
            target_link_pcl(${target})
        elseif(_lib_lower STREQUAL "boost")
            target_link_boost_all(${target})
        elseif(_lib_lower STREQUAL "osg")
            target_link_osg(${target})
        elseif(_lib_lower STREQUAL "osgearth")
            target_link_osgearth(${target})
        elseif(_lib_lower STREQUAL "gdal")
            target_link_gdal(${target})
        elseif(_lib_lower STREQUAL "eigen3" OR _lib_lower STREQUAL "eigen")
            target_link_eigen3(${target})
        elseif(_lib_lower STREQUAL "ceres")
            target_link_ceres(${target})
        elseif(_lib_lower STREQUAL "cgal")
            target_link_cgal(${target})
        elseif(_lib_lower STREQUAL "protobuf")
            target_link_protobuf(${target})
        elseif(_lib_lower STREQUAL "ncnn")
            target_link_ncnn(${target})
        elseif(_lib_lower STREQUAL "spdlog")
            target_link_spdlog(${target})
        elseif(_lib_lower STREQUAL "sqlite3" OR _lib_lower STREQUAL "sqlite")
            target_link_sqlite3(${target})
        elseif(_lib_lower STREQUAL "curl")
            target_link_curl(${target})
        elseif(_lib_lower STREQUAL "tiff")
            target_link_tiff(${target})
        elseif(_lib_lower STREQUAL "proj")
            target_link_proj(${target})
        elseif(_lib_lower STREQUAL "flann")
            target_link_flann(${target})
        elseif(_lib_lower STREQUAL "tbb")
            target_link_tbb(${target})
        elseif(_lib_lower STREQUAL "pugixml")
            target_link_pugixml(${target})
        elseif(_lib_lower STREQUAL "laslib")
            target_link_laslib(${target})
        elseif(_lib_lower STREQUAL "rapidjson")
            target_link_rapidjson(${target})
        elseif(_lib_lower STREQUAL "uuid")
            target_link_uuid(${target})
        elseif(_lib_lower STREQUAL "glslang")
            target_link_glslang(${target})
        elseif(_lib_lower STREQUAL "bit7z")
            target_link_bit7z(${target})
        elseif(_lib_lower STREQUAL "vcg")
            target_link_vcg(${target})
        elseif(_lib_lower STREQUAL "img")
            target_link_img(${target})
        else()
            message(WARNING "Unknown library: ${_lib}")
        endif()
    endforeach()
endmacro()

# =============================================================================
# 打印信息
# =============================================================================
function(thirdpart_print_info)
    message(STATUS "ThirdPart Configuration:")
    message(STATUS "  ROOT:        ${THIRDPART_ROOT}")
    message(STATUS "  INCLUDE:     ${THIRDPART_INCLUDE_DIR}")
    message(STATUS "  LIB:         ${THIRDPART_LIB_DIR}")
    message(STATUS "  LIB64:       ${THIRDPART_LIB64_DIR}")
    message(STATUS "  BIN:         ${THIRDPART_BIN_DIR}")
endfunction()

message(STATUS "ThirdPart toolchain loaded: ${THIRDPART_ROOT}")