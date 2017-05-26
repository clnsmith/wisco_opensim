# Settings.
# ---------
set(TOY_NAME "wipJointStiffness")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp *.c)

add_executable(${TOY_NAME} ${SOURCE_FILES})

target_link_libraries(${TOY_NAME} ${OpenSim_LIBRARIES})
target_link_libraries(${TOY_NAME} ${PLUGIN_NAME})

SET_TARGET_PROPERTIES (${TOY_NAME} PROPERTIES FOLDER Sandbox)
