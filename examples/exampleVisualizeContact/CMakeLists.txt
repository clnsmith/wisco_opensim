# Settings.
# ---------
set(EXAMPLE_NAME "exampleVisualizeContact")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${EXAMPLE_NAME} ${SOURCE_FILES})

target_link_libraries(${EXAMPLE_NAME} ${OpenSim_LIBRARIES})
target_link_libraries(${EXAMPLE_NAME} ${PLUGIN_NAME})

SET_TARGET_PROPERTIES (${EXAMPLE_NAME} PROPERTIES FOLDER Examples)

file(COPY inputs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/results)
