set(SOURCES "")
file(GLOB_RECURSE SOURCES "src/*.h" "src/*.cpp" "src/*.c")

add_library(PointCloudSegmentation ${SOURCES})
apply_compiler_config(PointCloudSegmentation)


# 头文件引用
target_include_directories(PointCloudSegmentation PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	CommonTools
	${Eigen3_INCLUDE_DIR}
	${OSG_INCLUDE_DIR} 
	${PCL_INCLUDE_DIR} 
	${VCG_INCLUDE_DIR}
	${Ceres_INCLUDE_DIR} 
)

# 依赖库引用
target_link_libraries(PointCloudSegmentation PUBLIC 
	CommonTools 
	OpenMP::OpenMP_CXX 
	${PCL_LIBS} 
	${CGAL_LIBS} 
	${Ceres_LIBRARY} 
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

install (TARGETS CommonTools
	RUNTIME DESTINATION ${BINDIR} COMPONENT Applications
	ARCHIVE DESTINATION ${LIBDIR} COMPONENT Libraries
	LIBRARY DESTINATION ${LIBDIR} COMPONENT Libraries)