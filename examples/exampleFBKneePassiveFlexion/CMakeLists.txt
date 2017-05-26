# Settings.
# ---------
set(EXAMPLE_NAME "exampleFBKneePassiveFlexion")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${EXAMPLE_NAME} ${SOURCE_FILES})

target_link_libraries(${EXAMPLE_NAME} ${OpenSim_LIBRARIES})

SET_TARGET_PROPERTIES (${EXAMPLE_NAME} PROPERTIES FOLDER Examples)

file(COPY inputs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
