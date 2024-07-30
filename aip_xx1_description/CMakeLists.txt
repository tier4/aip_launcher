cmake_minimum_required(VERSION 3.5)
project(aip_xx1_description)


find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(PythonInterp REQUIRED)
# Specify the path to your Python script
set(PYTHON_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/compile_xacro.py")
set(PYTHON_TEMPLATE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/templates")
set(PYTHON_CALIBRATION_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
set(PYTHON_XACRO_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/urdf")

# Run the Python script
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} ${PYTHON_SCRIPT} ${PYTHON_TEMPLATE_DIRECTORY} ${PYTHON_CALIBRATION_DIRECTORY} ${PYTHON_XACRO_DIRECTORY} ${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
)
# message(WARNING "Python script output: ${output}")

if(NOT result EQUAL 0)
    message(FATAL_ERROR "Python script failed with error: ${error}")
else()
    message(WARNING "Xacro Compiling Success")
endif()


ament_auto_package(INSTALL_TO_SHARE
    urdf
    config
)
