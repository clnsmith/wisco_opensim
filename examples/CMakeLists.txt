#Add all tests
#-------------
FILE(GLOB EXAMPLE_NAMES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/example*")
FOREACH(example_name ${EXAMPLE_NAMES})
    IF(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${example_name})
        add_subdirectory(${example_name})
    ENDIF()
ENDFOREACH(example_name)

#Copy Shared Folder to build
#---------------------------
#file(COPY shared DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
