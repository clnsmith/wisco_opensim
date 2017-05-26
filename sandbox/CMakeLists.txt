#Add all tests
#-------------
#FILE(GLOB TEST_NAMES RELATIVE "${WISCO_OPENSIM_DIR}/tests" "${WISCO_OPENSIM_DIR}/tests/*")
FILE(GLOB TEST_NAMES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/*")
FOREACH(test_name ${TEST_NAMES})
    IF(IS_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/${test_name})
        add_subdirectory(${test_name})
    ENDIF()
ENDFOREACH(test_name)
