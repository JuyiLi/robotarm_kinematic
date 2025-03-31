cmake_minimum_required(VERSION 3.5)
project(robotarm_kinematic)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)

SET(INCLUDE_DIR 

    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
include_directories(${INCLUDE_DIR})

SET(LIB_DIR 

    ${CMAKE_CURRENT_SOURCE_DIR}/lib
	
)
link_directories(${LIB_DIR})

SET(INCLUDE_FILES 

    ${CMAKE_CURRENT_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robotarm_parameters.h
)

add_library(${CMAKE_PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/${CMAKE_PROJECT_NAME}.cpp ${INCLUDE_FILES})


SET(test_list

    test
    )
foreach(var ${test_list})
    add_executable(${CMAKE_PROJECT_NAME}_${var} ${CMAKE_CURRENT_SOURCE_DIR}/example/${var}.cpp)

    target_link_libraries(${CMAKE_PROJECT_NAME}_${var} ${CMAKE_PROJECT_NAME})

endforeach()

add_custom_target(copyfile ALL
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${CMAKE_PROJECT_NAME}.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${CMAKE_PROJECT_NAME}.so
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "copy lib files to lib directory"
)

add_dependencies(copyfile ${CMAKE_PROJECT_NAME})

