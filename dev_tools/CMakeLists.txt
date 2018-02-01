#Add all subfolders
#-------------
# FILE(GLOB TOY_NAMES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/*")
# FOREACH(toy_name ${TOY_NAMES})
#     IF(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${toy_name})
#         add_subdirectory(${toy_name})
#     ENDIF()
# ENDFOREACH(toy_name)

#Create WISCO_Tools library
set(WISCO_TOOLS_NAME "WISCO_Tools"
CACHE STRING "Name of static library to create")



#Specify Files
file(GLOB SOURCE_FILES */*.cpp */*.c)
file(GLOB INCLUDE_FILES */*.h)

#Specify Plugin Details
add_library(${WISCO_TOOLS_NAME} STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
target_link_libraries(${WISCO_TOOLS_NAME} ${OpenSim_LIBRARIES})
set_target_properties(${WISCO_TOOLS_NAME} PROPERTIES DEBUG_POSTFIX "_d")
SET_TARGET_PROPERTIES(${WISCO_TOOLS_NAME} PROPERTIES FOLDER WISCO_Tools)
install(TARGETS ${WISCO_TOOLS_NAME} DESTINATION wisco_tools)
