
add_executable(AppStarter main.cpp)

apply_compiler_config(AppStarter)

# 头文件引用
target_include_directories(AppStarter PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR} 
    PointCloudManager
)

# 依赖库引用
target_link_libraries(AppStarter PUBLIC 
    PointCloudManager 
    ${Boost_LIBRARIES} 
	${OSG_LIBS} 
	${OSGEARTH_LIBS}
	${PCL_LIBS} 
	${CGAL_LIBS} 
	${Ceres_LIBRARY} 
	${ncnn_LIBRARY} 
)

set_target_properties(AppStarter PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
	LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
	ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

set(BINDIR "bin")
set(LIBDIR "lib")

install (TARGETS AppStarter
	RUNTIME DESTINATION ${BINDIR} COMPONENT Applications
	ARCHIVE DESTINATION ${LIBDIR} COMPONENT Libraries
	LIBRARY DESTINATION ${LIBDIR} COMPONENT Libraries)
