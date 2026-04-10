file(GLOB srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/TransformFunctions/*.c		# cfft and rfft functions
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/ComplexMathFunctions/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/CommonTables/*.c
)

add_library(CMSIS-DSP STATIC ${srcs})

target_include_directories(CMSIS-DSP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Include/
)

target_include_directories(CMSIS-DSP PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Include/
    ${CMAKE_CURRENT_SOURCE_DIR}/PrivateInclude/
)

target_link_libraries(CMSIS-DSP PRIVATE
    m
)

target_compile_definitions(CMSIS-DSP PRIVATE
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
    __FPU_PRESENT=1
)
