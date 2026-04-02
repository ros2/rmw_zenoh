# Rust version 1.75 requires setting a special CMake variable to modify Zenoh C's Cargo.lock
# This function sets $FLAGS_FOR_CARGO_VERSION in parent scope equal to a -D argument
# for that flag.
function(get_flags_for_cargo_version)
    find_program(CARGO_EXECUTABLE cargo)

    set(FLAGS_FOR_CARGO_VERSION "" PARENT_SCOPE)

    if(CARGO_EXECUTABLE)
        execute_process(
            COMMAND ${CARGO_EXECUTABLE} --version
            OUTPUT_VARIABLE CARGO_VERSION_RAW
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(CARGO_VERSION_RAW MATCHES "cargo ([0-9]+\\.[0-9]+\\.[0-9]+)")
            set(RUST_VERSION "${CMAKE_MATCH_1}")

            # There appears only to be 1.75.0, but allow all 1.75.x a new release happens
            if(RUST_VERSION VERSION_GREATER_EQUAL "1.75" AND RUST_VERSION VERSION_LESS "1.76")
                set(FLAGS_FOR_CARGO_VERSION "-DZENOHC_MSRV_1_75=TRUE" PARENT_SCOPE)
                message(STATUS "Rust 1.75 detected; setting ZENOHC_MSRV_1_75=TRUE.")
            endif()
        endif()
    else()
        message(STATUS "cargo not found; FLAGS_FOR_CARGO_VERSION remains empty.")
    endif()
endfunction()