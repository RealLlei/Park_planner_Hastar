# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)
set(CMAKE_CXX_STANDARD 17)
# 生成链接库
add_library(BasicFunciton ${DIR_SRCS})

# 如果 OpenMP 在主 CMakeLists.txt 中被成功配置，并且 OpenMP_LIBRARIES 被设置
# BasicFunciton 目标如果直接使用了 OpenMP 功能，也需要链接 OpenMP 库。
if(OpenMP_CONFIGURED_SUCCESSFULLY AND OpenMP_LIBRARIES)
  target_link_libraries(BasicFunciton INTERFACE ${OpenMP_LIBRARIES})
endif()