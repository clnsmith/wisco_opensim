#Add all toys
#-------------
FILE(GLOB TOY_NAMES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/*")
FOREACH(toy_name ${TOY_NAMES})
    IF(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${toy_name})
        add_subdirectory(${toy_name})
    ENDIF()
ENDFOREACH(toy_name)
