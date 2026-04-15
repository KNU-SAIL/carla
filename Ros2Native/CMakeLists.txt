cmake_minimum_required (
  VERSION
  ${CARLA_CMAKE_MINIMUM_REQUIRED_VERSION}
)

project (carla-ros2-native-project)

include (ExternalProject)

set (PROJECT_INSTALL_PATH ${CMAKE_CURRENT_BINARY_DIR}/install)

ExternalProject_add (
  foonathan_memory
  URL https://github.com/eProsima/foonathan_memory_vendor/archive/${CARLA_FOONATHAN_MEMORY_VENDOR_TAG}.zip
  # vendor CMakeLists.txt의 "if(NOT foonathan_memory_FOUND)" 조건을 "if(TRUE)"로 패치하여
  # 시스템의 foonathan_memory(0.7.1)를 발견하더라도 항상 vendored GIT_TAG v0.7-3을 소스 빌드함.
  PATCH_COMMAND sed -i "s/if(NOT foonathan_memory_FOUND)/if(TRUE)/" CMakeLists.txt
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_PATH}
    # UE5 Clang toolchain은 전달하지 않음: foonathan/memory v0.7-3의
    # get_container_node_sizes CMake 테스트가 -stdlib=libc++ 환경에서 실패함.
    -DBUILD_SHARED_LIBS=ON
)

# Note: We are using GIT_REPOSITORY instead URL for fastdds because fastdss contains submodules
#       and we need the .git folders to download the submodules
ExternalProject_add (
  fastdds
  GIT_REPOSITORY https://github.com/eProsima/Fast-DDS.git
  GIT_TAG ${CARLA_FASTDDS_TAG}
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_PATH}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_TESTING=OFF
    -DCOMPILE_EXAMPLES=OFF
    -DCOMPILE_TOOLS=OFF
    -DTHIRDPARTY_Asio=FORCE
    -DTHIRDPARTY_fastcdr=FORCE
    -DTHIRDPARTY_TinyXML2=FORCE
    -DSQLITE3_SUPPORT=OFF
    -DOPENSSL_FOUND:BOOL=ON
    -DOPENSSL_INCLUDE_DIR:FILEPATH=${UE_OPENSSL_INCLUDE}
    -DOPENSSL_SSL_LIBRARY:FILEPATH=${UE_OPENSSL_LIBS}/libssl.a
    -DOPENSSL_CRYPTO_LIBRARY:FILEPATH=${UE_OPENSSL_LIBS}/libcrypto.a
  DEPENDS foonathan_memory
)

ExternalProject_Add (
  carla-ros2-native-lib
  DEPENDS fastdds
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/LibCarlaRos2Native
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_PATH}
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
)

set (CARLA_PLUGIN_BINARY_PATH ${CMAKE_SOURCE_DIR}/Unreal/CarlaUnreal/Plugins/Carla/Binaries/Linux)

make_directory (${CARLA_PLUGIN_BINARY_PATH})

add_custom_command (
  TARGET carla-ros2-native-lib
  POST_BUILD
  COMMAND
    ${CMAKE_COMMAND} -E copy
    ${PROJECT_INSTALL_PATH}/lib/*.so*
    ${CARLA_PLUGIN_BINARY_PATH}
)

carla_add_custom_target (
  carla-ros2-native
  "Build the ROS2-Native CARLA subproject."
  DEPENDS carla-ros2-native-lib
)
