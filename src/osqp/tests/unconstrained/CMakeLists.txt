get_directory_property(OSQP_TESTCASE_SRCS DIRECTORY ${PROJECT_SOURCE_DIR}/tests DEFINITION OSQP_TESTCASE_SRCS)

set(OSQP_TESTCASE_SRCS
    ${OSQP_TESTCASE_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_unconstrained.cpp
    PARENT_SCOPE)

get_directory_property(OSQP_TESTCASE_GENERATED_SRCS DIRECTORY ${PROJECT_SOURCE_DIR}/tests DEFINITION OSQP_TESTCASE_GENERATED_SRCS)

set(OSQP_TESTCASE_GENERATED_SRCS
    ${OSQP_TESTCASE_GENERATED_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/unconstrained_data.cpp
    PARENT_SCOPE)

get_directory_property(OSQP_TESTCASE_GENERATORS DIRECTORY ${PROJECT_SOURCE_DIR}/tests DEFINITION OSQP_TESTCASE_GENERATORS)

set(OSQP_TESTCASE_GENERATORS
    ${OSQP_TESTCASE_GENERATORS}
    ${CMAKE_CURRENT_SOURCE_DIR}/generate_problem.py
    PARENT_SCOPE)

get_directory_property(OSQP_TESTCASE_DIRS DIRECTORY ${PROJECT_SOURCE_DIR}/tests DEFINITION OSQP_TESTCASE_DIRS)

set(OSQP_TESTCASE_DIRS
    ${OSQP_TESTCASE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    PARENT_SCOPE)
