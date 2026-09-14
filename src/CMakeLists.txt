# C 与 C++ 源码都纳入编译, 递归包含 framework/、tests/ 子目录
# framework/include 注册为公共头文件目录, 所有 .cpp 直接 #include "xxx.h"
FILE(GLOB_RECURSE app_sources
    ${CMAKE_SOURCE_DIR}/src/*.c
    ${CMAKE_SOURCE_DIR}/src/*.cpp)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS framework/include)
