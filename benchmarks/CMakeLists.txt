# This function set up benchmarks to ensure they are always running at maximum optimization
function(pcr_add_benchmark TARGET_NAME)
    add_executable(${TARGET_NAME} ${ARGN})

    # Common includes
    target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

    # Add pcr library
    target_link_libraries(${TARGET_NAME} PRIVATE pcr benchmark::benchmark)

    # Force release-level optimizations
    if (MSVC)
        target_compile_options(${TARGET_NAME} PRIVATE /O2)
    else ()
        target_compile_options(${TARGET_NAME} PRIVATE -O3)
    endif ()


    # Set project warnings
    if (COMMAND set_project_warnings)
        set_project_warnings(${TARGET_NAME})
    endif ()

    set_target_properties(${TARGET_NAME} PROPERTIES
            CODE_ATTRIBUTE_CODE_SIGN_INJECT_BASE_ENTITLEMENTS "YES"
    )

    message(STATUS "Configured benchmark ${TARGET_NAME} with forced optimization.")
endfunction()

# Fetch & find google benchmark
fetch_google_benchmark()

# Add benchmark subdirectories
add_subdirectory(io)
add_subdirectory(spatial)
