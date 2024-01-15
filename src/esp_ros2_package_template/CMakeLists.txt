cmake_minimum_required(VERSION 3.5)
project(project)

# Include the ExternalProject module
include(ExternalProject)
ExternalProject_Add(
    ${PROJECT_NAME}
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/main
    BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND idf.py build
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
)
