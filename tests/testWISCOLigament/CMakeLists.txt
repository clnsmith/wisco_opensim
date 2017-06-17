# Settings.
# ---------
set(TEST_NAME "testWISCOLigament")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${TEST_NAME} ${SOURCE_FILES})

target_link_libraries(${TEST_NAME} ${OpenSim_LIBRARIES})
target_link_libraries(${TEST_NAME} ${PLUGIN_NAME})

SET_TARGET_PROPERTIES (${TEST_NAME} PROPERTIES FOLDER Tests)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/results)
