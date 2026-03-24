# This function set up benchmarks to ensure they are always running at maximum optimization
function(pcr_add_benchmark TARGET_NAME)
    add_executable(${TARGET_NAME} ${ARGN})

    # Common includes
    target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

    # Add pcr library
    target_link_libraries(${TARGET_NAME} PRIVATE pcr benchmark::benchmark)


    # Set project warnings
    if (COMMAND set_project_warnings)
        set_project_warnings(${TARGET_NAME})
    endif ()

    if (APPLE)
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND codesign --force --entitlements ${CMAKE_SOURCE_DIR}/entitlements.plist -s - $<TARGET_FILE:${TARGET_NAME}>
                COMMENT "Signing with get-task-allow for Processor Trace"
        )
    endif ()

    message(STATUS "Configured benchmark ${TARGET_NAME} with forced optimization.")
endfunction()

# Fetch & find google benchmark
fetch_google_benchmark()

# Add benchmark subdirectories
add_subdirectory(io)
add_subdirectory(spatial)
