# Settings.
# ---------
set(TOY_NAME "wipCOMAKMain")
# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${TOY_NAME} ${SOURCE_FILES})

target_link_libraries(${TOY_NAME} ${OpenSim_LIBRARIES})
target_link_libraries(${TOY_NAME} ${PLUGIN_NAME})
target_link_libraries(${TOY_NAME} ${WISCO_TOOLS_NAME})

SET_TARGET_PROPERTIES (${TOY_NAME} PROPERTIES FOLDER Sandbox)

file(COPY inputs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
