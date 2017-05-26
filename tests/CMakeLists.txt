#Add all tests
#-------------
FILE(GLOB TEST_NAMES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/test*")
FOREACH(test_name ${TEST_NAMES})
    IF(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${test_name})
        add_subdirectory(${test_name})
    ENDIF()
ENDFOREACH(test_name)

#Copy Shared Folder to build
#---------------------------
file(COPY shared DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
