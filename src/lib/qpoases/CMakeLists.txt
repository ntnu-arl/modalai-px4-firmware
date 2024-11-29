include_directories(${CMAKE_CURRENT_SOURCE_DIR}/qpOASES/include)

# Disable warnings
add_compile_options(-Wno-float-equal)
add_compile_options(-Wno-shadow)
add_compile_options(-Wno-double-promotion)

file(GLOB QPOASES_SRC ${CMAKE_CURRENT_SOURCE_DIR}/qpOASES/src/*.cpp)

# # Filter out tests as they cause errors
# list(FILTER QPOASES_SRC EXCLUDE REGEX ".*_test.*\\.cc$")

px4_add_library(qpoases ${QPOASES_SRC})

target_include_directories(qpoases PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/qpOASES/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)