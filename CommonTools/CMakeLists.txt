set(SOURCES "")

aux_source_directory(cstring STRING_SRC)
aux_source_directory(Data DATA_SRC)
aux_source_directory(include INC_SRC)
aux_source_directory(Tool STRING_SRC)

list(APPEND SOURCES ${STRING_SRC} 
                    ${DATA_SRC} 
                    ${STRING_SRC}
					${INC_SRC})

add_library(CommonTools ${SOURCES})
apply_compiler_config(CommonTools)

# target_include_directories(CommonTools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(CommonTools PUBLIC ${DEPENDENCY_LIBS})

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
