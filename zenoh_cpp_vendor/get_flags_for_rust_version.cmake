# Rust version 1.75 requires setting a special CMake variable to modify Zenoh C's Cargo.lock
# This function sets $FLAGS_FOR_RUST_VERSION in parent scope equal to a -D argument
# for that flag.
function(get_flags_for_rust_version)
    find_program(RUSTC_EXECUTABLE rustc)

    set(FLAGS_FOR_RUST_VERSION "" PARENT_SCOPE)

    if(RUSTC_EXECUTABLE)
        execute_process(
            COMMAND ${RUSTC_EXECUTABLE} --version
            OUTPUT_VARIABLE RUSTC_VERSION_RAW
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(RUSTC_VERSION_RAW MATCHES "rustc ([0-9]+\\.[0-9]+\\.[0-9]+)")
            set(RUST_VERSION "${CMAKE_MATCH_1}")

            # There appears only to be 1.75.0, but allow all 1.75.x in case
            # such a release happens in the future.
            if(RUST_VERSION VERSION_GREATER_EQUAL "1.75" AND RUST_VERSION VERSION_LESS "1.76")
                set(FLAGS_FOR_RUST_VERSION "-DZENOHC_MSRV_1_75=TRUE" PARENT_SCOPE)
                message(STATUS "Rust 1.75 detected; setting ZENOHC_MSRV_1_75=TRUE.")
            endif()
        endif()
    else()
        message(STATUS "rustc not found; FLAGS_FOR_RUST_VERSION remains empty.")
    endif()
endfunction()