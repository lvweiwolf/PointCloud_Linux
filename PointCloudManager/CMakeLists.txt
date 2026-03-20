set(SOURCES "")
aux_source_directory(include INC_SRC)
aux_source_directory(BusinessNode BUSSINESS_SRC)
aux_source_directory(LasFile LAS_SRC)
aux_source_directory(ProjectManager PROJECT_SRC)
aux_source_directory(Segment SEGMENT_SRC)

list(APPEND SOURCES ${INC_SRC} 
					${BUSSINESS_SRC} 
                    ${LAS_SRC} 
                    ${PROJECT_SRC} 
					${SEGMENT_SRC})

add_library(PointCloudManager SHARED 
	${SOURCES}
)

apply_compiler_config(PointCloudManager)

target_compile_definitions(PointCloudManager PRIVATE 
	POINTCLOUDMANAGERAPI
	PCL_NO_PRECOMPILE
	PCL_ONLY_CORE_POINT_TYPES
	INFERENCE_USE_NCNN 
	CPL_DISABLE_DLL 
)

# 头文件引用
target_include_directories(PointCloudManager PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	CommonTools
	PointCloudSegmentation
	${THIRDPART_INCLUDE_DIR}
	${TBB_INCLUDE_DIR}
)

# 依赖库引用
target_link_libraries(PointCloudManager PUBLIC 
	OpenMP::OpenMP_CXX 
	Vulkan::Vulkan
	CommonTools 
	PointCloudSegmentation 
	${TBB_LIBRARIES}
)

set_target_properties(PointCloudManager PROPERTIES
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

install (TARGETS PointCloudManager
	RUNTIME DESTINATION ${BINDIR} COMPONENT Applications
	ARCHIVE DESTINATION ${LIBDIR} COMPONENT Libraries
	LIBRARY DESTINATION ${LIBDIR} COMPONENT Libraries)