cmake_minimum_required(VERSION 3.9)

project(hybrid_a_star LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

#set DEBUG
set(CMAKE_BUILD_TYPE DEBUG) 

# 检查编译器是否为 AppleClang
if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  message(STATUS "AppleClang detected. Checking for libomp from Homebrew.")
  set(LIBOMP_PREFIX "/usr/local/opt/libomp") # Homebrew prefix for libomp
  set(OPENMP_INCLUDE_DIR "${LIBOMP_PREFIX}/include")
  set(OPENMP_LIB_DIR "${LIBOMP_PREFIX}/lib")
  
  if(EXISTS "${OPENMP_INCLUDE_DIR}/omp.h" AND EXISTS "${OPENMP_LIB_DIR}/libomp.dylib")
    message(STATUS "Found libomp at ${LIBOMP_PREFIX}. Configuring OpenMP for AppleClang.")
    include_directories(SYSTEM "${OPENMP_INCLUDE_DIR}")
    
    # Apple Clang 使用 -Xclang -fopenmp 进行编译
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xclang -fopenmp")
    
    set(OpenMP_LIBRARIES omp CACHE INTERNAL "OpenMP libraries for AppleClang")
    link_directories(SYSTEM "${OPENMP_LIB_DIR}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${OPENMP_LIB_DIR} -lomp")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -L${OPENMP_LIB_DIR} -lomp")
    set(OpenMP_CONFIGURED_SUCCESSFULLY TRUE CACHE INTERNAL "OpenMP configured via libomp for AppleClang")
    
  else()
    message(FATAL_ERROR "libomp from Homebrew not found or incomplete at ${LIBOMP_PREFIX}. "
                        "Please run 'brew install libomp'.")
    set(OpenMP_CONFIGURED_SUCCESSFULLY FALSE CACHE INTERNAL "OpenMP configuration failed")
  endif()
  
else() 
  # 对于非 AppleClang 的编译器，尝试标准的 find_package(OpenMP)
  message(STATUS "Non-AppleClang compiler. Attempting standard OpenMP detection.")
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "Standard OpenMP found.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(OpenMP_LIBRARIES ${OpenMP_CXX_LIBRARIES} CACHE INTERNAL "Standard OpenMP libraries")
    set(OpenMP_CONFIGURED_SUCCESSFULLY TRUE CACHE INTERNAL "Standard OpenMP configured")
  else()
    message(WARNING "Standard OpenMP not found. OpenMP features will be disabled.")
    set(OpenMP_CONFIGURED_SUCCESSFULLY FALSE CACHE INTERNAL "Standard OpenMP configuration failed")
  endif()
endif()

# --- Python 配置 ---
find_package(Python3 3.13.2 COMPONENTS Interpreter Development NumPy REQUIRED)

if(Python3_FOUND)
  message(STATUS "Found Python 3 (${Python3_VERSION_STRING})")
  message(STATUS "  Interpreter: ${Python3_EXECUTABLE}")
  message(STATUS "  Libraries: ${Python3_LIBRARIES}")
  message(STATUS "  Include Dirs: ${Python3_INCLUDE_DIRS}")
  if(Python3_NumPy_FOUND)
    message(STATUS "  NumPy Include Dirs: ${Python3_NumPy_INCLUDE_DIRS}")
    include_directories(SYSTEM "${Python3_NumPy_INCLUDE_DIRS}")
  else()
    message(WARNING "NumPy not found by CMake's FindPython3. Plotting might fail.")
  endif()
  include_directories(SYSTEM "${Python3_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Python 3.9 or newer with Development headers and NumPy could not be found.")
endif()

aux_source_directory(. DIR_SRCS_MAIN) 

add_subdirectory(src)  

add_subdirectory(result_plot) 

add_executable(test main.cc read_conf.cc)

aux_source_directory(. source_list)

if(OpenMP_CONFIGURED_SUCCESSFULLY AND OpenMP_LIBRARIES)
  target_link_libraries(test BasicFunciton ResultPlot ${OpenMP_LIBRARIES})
else()
  target_link_libraries(test BasicFunciton ResultPlot)
endif()

# RPATH 设置
set(CONDA_LIB_DIR "")
if(DEFINED ENV{CONDA_PREFIX})
  set(CONDA_LIB_DIR "$ENV{CONDA_PREFIX}/lib")
  message(STATUS "Conda prefix detected: $ENV{CONDA_PREFIX}")
  message(STATUS "Adding Conda lib directory to RPATH: ${CONDA_LIB_DIR}")
  set_target_properties(test PROPERTIES
      INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;${CONDA_LIB_DIR}"
      BUILD_RPATH "$ORIGIN;$ORIGIN/../lib;${CONDA_LIB_DIR}"
      MACOSX_RPATH ON
  )
else()
  message(STATUS "CONDA_PREFIX environment variable not set. Assuming system/non-conda Python.")
  if(Python3_FOUND AND Python3_INCLUDE_DIRS)
      # 对于非Conda的Framework Python (如系统或Xcode自带的)
      string(REGEX REPLACE "/Python3\\.framework/Versions/[0-9.]+.*$" "" _python_frameworks_dir "${Python3_INCLUDE_DIRS}")
      if(IS_DIRECTORY "${_python_frameworks_dir}/Python3.framework")
          message(STATUS "Deduced Python Frameworks directory for RPATH (non-conda): ${_python_frameworks_dir}")
          set_target_properties(test PROPERTIES
              INSTALL_RPATH "${_python_frameworks_dir}"
              BUILD_RPATH "${_python_frameworks_dir}"
              MACOSX_RPATH ON
          )
      else()
          message(WARNING "Could not deduce non-conda Python Frameworks directory from Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}.")
      endif()
  endif()
endif()

set_target_properties(test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# 定义配置文件的源目录和目标目录
set(CONFIG_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config")
set(CONFIG_DEST_DIR_IN_BIN "${CMAKE_BINARY_DIR}/bin/config")

# 在构建 test 目标之后，执行复制操作
add_custom_command(
    TARGET test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CONFIG_DEST_DIR_IN_BIN}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different 
            "${CONFIG_SOURCE_DIR}/hya_param.conf" 
            "${CONFIG_DEST_DIR_IN_BIN}/hya_param.conf"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different 
            "${CONFIG_SOURCE_DIR}/parking_scenario.txt" 
            "${CONFIG_DEST_DIR_IN_BIN}/parking_scenario.txt"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different 
            "${CONFIG_SOURCE_DIR}/vehicle_param.conf" 
            "${CONFIG_DEST_DIR_IN_BIN}/vehicle_param.conf"
    COMMENT "Copying configuration files to ${CONFIG_DEST_DIR_IN_BIN}"
)