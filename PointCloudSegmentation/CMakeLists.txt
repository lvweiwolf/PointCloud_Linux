set(SOURCES "")
file(GLOB_RECURSE SOURCES "src/*.h" "src/*.cpp" "src/*.c")

add_library(PointCloudSegmentation SHARED 
	${SOURCES}
)

apply_compiler_config(PointCloudSegmentation)

target_compile_definitions(PointCloudSegmentation PRIVATE 
	POINTCLOUDSEGMENTAPI
	PCL_NO_PRECOMPILE
	PCL_ONLY_CORE_POINT_TYPES
	INFERENCE_USE_NCNN 
	CPL_DISABLE_DLL 
	_USE_MATH_DEFINES 
	GOOGLE_GLOG_DLL_DECL
)

# 头文件引用
target_include_directories(PointCloudSegmentation PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	CommonTools
	${Eigen3_INCLUDE_DIR}
	${PCL_INCLUDE_DIR} 
	${LASlib_INCLUDE_DIR} 
	${Boost_INCLUDE_DIR} 
	${bit7z_INCLUDE_DIR}
	${THIRDPART_INCLUDE_DIR}
)

set(OpenCV_Part_Libs
    opencv_core
	opencv_imgproc
	opencv_dnn
	opencv_flann
	opencv_imgcodecs
)

set(Boost_Part_Libs
	boost_locale
	boost_system
	boost_filesystem
)

set(PCL_Part_Libs
	pcl_io
	pcl_io_ply
	pcl_kdtree
	pcl_keypoints
	pcl_ml
	pcl_octree
	pcl_search
	pcl_recognition
	pcl_filters
	pcl_segmentation
	pcl_common
	pcl_features
)

set(CGAL_Part_Libs
	CGAL 
	CGAL_Core 
	gmp
	mpfr
)

# 依赖库引用
target_link_libraries(PointCloudSegmentation PUBLIC 
	OpenMP::OpenMP_CXX 
	Vulkan::Vulkan
	CommonTools 
	${Boost_Part_Libs} 
	${PCL_Part_Libs} 
	${CGAL_Part_Libs} 
	${OpenCV_Part_Libs} 
	${OSG_LIBS} 
	${OSGEARTH_LIBS}
	${Ceres_LIBRARY} 
	${ncnn_LIBRARY} 
	${bit7z_LIBRARY} 
	${LASlib_LIBRARY}
	${UUID_LIBRARY} 
)

set_target_properties(PointCloudSegmentation PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
	LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
	ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

set(BINDIR "bin")
set(LIBDIR "lib")

# 打包安装
install(DIRECTORY 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

install (TARGETS PointCloudSegmentation
	RUNTIME DESTINATION ${BINDIR} COMPONENT Applications
	ARCHIVE DESTINATION ${LIBDIR} COMPONENT Libraries
	LIBRARY DESTINATION ${LIBDIR} COMPONENT Libraries)