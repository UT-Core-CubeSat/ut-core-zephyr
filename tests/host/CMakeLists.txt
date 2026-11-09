cmake_minimum_required(VERSION 3.20)
project(soh_host_tests C)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Repo root is two folders up from tests/host
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(test_soh
    test_soh.c
    ${REPO_ROOT}/common/soh.c
)

target_include_directories(test_soh PRIVATE ${REPO_ROOT})
target_compile_options(test_soh PRIVATE -Wall -Wextra -Werror)

add_test(NAME test_soh COMMAND test_soh)