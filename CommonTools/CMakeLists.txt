set(SOURCES "")

aux_source_directory(include INC_SRC)
aux_source_directory(cstring STRING_SRC)
aux_source_directory(Data DATA_SRC)
aux_source_directory(Tool STRING_SRC)

list(APPEND SOURCES ${INC_SRC} 
					${STRING_SRC} 
                    ${DATA_SRC} 
                    ${STRING_SRC})

add_library(CommonTools SHARED 
	 ${SOURCES}
)

apply_compiler_config(CommonTools)

target_compile_definitions(CommonTools PRIVATE 
	COMMONTOOLSAPI 
)

# 头文件引用
target_include_directories(CommonTools PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	${OSG_INCLUDE_DIR}
)

# 依赖库引用
target_link_libraries(CommonTools PUBLIC 
	OpenMP::OpenMP_CXX 
	pugixml::pugixml
	${OSG_LIBS}
)

set_target_properties(CommonTools PROPERTIES
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
