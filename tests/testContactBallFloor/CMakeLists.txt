# Settings.
# ---------
set(TEST_NAME "testContactBallFloor")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${TEST_NAME} ${SOURCE_FILES})

target_link_libraries(${TEST_NAME} ${OpenSim_LIBRARIES})

SET_TARGET_PROPERTIES (${TEST_NAME} PROPERTIES FOLDER Tests)

file(COPY inputs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/results)
