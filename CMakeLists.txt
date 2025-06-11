cmake_minimum_required(VERSION 3.0.0)

project("ClothOpt")

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Compiler options
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 ) # Emit a compile flags file to support completion engines 

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using Clang (linux or apple) or GCC
  message("Using clang/gcc compiler flags")
  SET(BASE_CXX_FLAGS "-std=c++17 -Wall -Wextra -g3")
  SET(DISABLED_WARNINGS " -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-deprecated-declarations -Wno-missing-braces")
  SET(TRACE_INCLUDES " -H -Wno-error=unused-command-line-argument")

  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    message("Setting clang-specific options")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -ferror-limit=5 -fcolor-diagnostics")
    SET(CMAKE_CXX_FLAGS_DEBUG          "-fsanitize=address -fno-limit-debug-info")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -fmax-errors=5")
    message("Setting gcc-specific options")
    SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} -Wno-maybe-uninitialized -Wno-format-zero-length -Wno-unused-but-set-parameter -Wno-unused-but-set-variable")
  endif()

  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TRACE_INCLUDES}") # uncomment if you need to track down where something is getting included from
  SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -g3")
  SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELEASE        "-march=native -O3 -DNDEBUG -DTBB_USE_THREADING_TOOLS")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
  message("Using Visual Studio compiler flags")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") # parallel build
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4267\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4244\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4305\"")  # ignore truncation on initialization
  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")

  add_definitions(/D "_CRT_SECURE_NO_WARNINGS")
  add_definitions(-DNOMINMAX)
  add_definitions(-D_USE_MATH_DEFINES)

else()
  # unrecognized
  message( FATAL_ERROR "Unrecognized compiler [${CMAKE_CXX_COMPILER_ID}]" )
endif()

# === PERFORMANCE LIBRARIES ===

# Find Eigen
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Find Intel oneTBB for parallel processing
find_package(TBB CONFIG QUIET)
if(TBB_FOUND)
    message(STATUS "Found Intel oneTBB: ${TBB_VERSION}")
    set(HAVE_TBB TRUE)
else()
    # Try alternative TBB package names
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(TBB tbb)
        if(TBB_FOUND)
            message(STATUS "Found TBB via pkg-config: ${TBB_VERSION}")
            set(HAVE_TBB TRUE)
        endif()
    endif()
    
    if(NOT TBB_FOUND)
        # Try system installation
        find_path(TBB_INCLUDE_DIR tbb/tbb.h 
            PATHS /usr/include /usr/local/include /opt/intel/oneapi/tbb/latest/include)
        find_library(TBB_LIBRARY tbb 
            PATHS /usr/lib /usr/local/lib /opt/intel/oneapi/tbb/latest/lib/intel64/gcc4.8)
        
        if(TBB_INCLUDE_DIR AND TBB_LIBRARY)
            message(STATUS "Found TBB manually: ${TBB_LIBRARY}")
            set(HAVE_TBB TRUE)
            # Create imported target for manual find
            add_library(TBB::tbb UNKNOWN IMPORTED)
            set_target_properties(TBB::tbb PROPERTIES
                IMPORTED_LOCATION "${TBB_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${TBB_INCLUDE_DIR}")
        else()
            message(WARNING "Intel oneTBB not found. Parallel collision detection will be disabled.")
            message(STATUS "To install TBB on Ubuntu: sudo apt install libtbb-dev")
            message(STATUS "To install TBB on macOS: brew install tbb")
            set(HAVE_TBB FALSE)
        endif()
    endif()
endif()

# Find OpenMP as fallback
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    message(STATUS "Found OpenMP: ${OpenMP_CXX_VERSION}")
    set(HAVE_OPENMP TRUE)
else()
    message(WARNING "OpenMP not found. Parallel processing will be limited.")
    set(HAVE_OPENMP FALSE)
endif()

# Add Openvdb
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)

# Add polyscope
message("\n\n == CMAKE recursively building Polyscope\n")
add_subdirectory("deps/polyscope")

# Add IMath
add_subdirectory("deps/Imath")

# Add Alembic and set for Options
set(ALEMBIC_BUILD_TESTS OFF CACHE BOOL "Disable Alembic's tests")
set(ALEMBIC_LIB_USES_TESTS OFF CACHE BOOL "Disable Alembic's tests")
set(ALEMBIC_SHARED_LIBS OFF)  # Build primary Alembic library as static
set(USE_HDF5 OFF)              # Enable HDF5 support
set(USE_MAYA OFF)              # Enable MAYA
set(USE_PYALEMBIC OFF)         # Build the boost python bindings for Alembic
add_subdirectory("deps/alembic")

file(GLOB_RECURSE SRC_CPP
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

set(SHARED_SOURCES ${SRC_CPP})

SET(SRC_FILES ${CPP_FILES} ${SIM_FILES})

find_package(LibXml2 REQUIRED)

# Function to define an executable with common includes and linking
function(add_main_executable target_name source_file)
    add_executable(${target_name}
        ${source_file}
        ${SHARED_SOURCES}
    )

    target_include_directories(${target_name} PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/deps/libigl/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/deps/fbx/include"
        ${LIBXML2_INCLUDE_DIR}
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/deps/alembic/lib"
        "${CMAKE_CURRENT_SOURCE_DIR}/deps/eigen"
        "${CMAKE_CURRENT_SOURCE_DIR}/deps/json"
        "${CMAKE_CURRENT_SOURCE_DIR}/deps"
        ${OpenCASCADE_INCLUDE_DIR}
    )

    # Base libraries
    target_link_libraries(${target_name}
        polyscope
        Eigen3::Eigen
        Alembic
        OpenVDB::openvdb
    )

    # Add parallel processing libraries
    if(HAVE_TBB)
        target_link_libraries(${target_name} TBB::tbb)
        target_compile_definitions(${target_name} PRIVATE HAVE_TBB=1)
        message(STATUS "Linking ${target_name} with Intel oneTBB")
    elseif(HAVE_OPENMP)
        target_link_libraries(${target_name} OpenMP::OpenMP_CXX)
        target_compile_definitions(${target_name} PRIVATE HAVE_OPENMP=1)
        message(STATUS "Linking ${target_name} with OpenMP")
    else()
        target_compile_definitions(${target_name} PRIVATE NO_PARALLEL=1)
        message(STATUS "Building ${target_name} without parallel processing")
    endif()

    add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
endfunction()

# === Main Executables ===
add_main_executable(Visualization visual.cpp)
add_main_executable(Simulation simulation.cpp)
add_main_executable(Optimization optimization.cpp)

# Print configuration summary
message(STATUS "")
message(STATUS "=== ClothOpt Configuration Summary ===")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
if(HAVE_TBB)
    message(STATUS "Parallel processing: Intel oneTBB ✓")
elseif(HAVE_OPENMP)
    message(STATUS "Parallel processing: OpenMP ✓")
else()
    message(STATUS "Parallel processing: None (performance limited)")
endif()
message(STATUS "=====================================")
message(STATUS "")