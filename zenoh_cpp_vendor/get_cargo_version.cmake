# Function: get_cargo_version
# Sets the CARGO_VERSION variable in the parent scope (e.g., "1.75.0")
function(get_cargo_version)
    find_program(CARGO_EXECUTABLE cargo)

    if(NOT CARGO_EXECUTABLE)
        message(WARNING "Cargo executable not found. CARGO_VERSION will be unset.")
        set(CARGO_VERSION "NOTFOUND" PARENT_SCOPE)
        return()
    endif()

    execute_process(
        COMMAND ${CARGO_EXECUTABLE} --version
        OUTPUT_VARIABLE CARGO_VERSION_RAW
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Regex captures the standard 'cargo X.Y.Z' format
    if(CARGO_VERSION_RAW MATCHES "cargo ([0-9]+\\.[0-9]+\\.[0-9]+)")
        set(CARGO_VERSION "${CMAKE_MATCH_1}" PARENT_SCOPE)
    else()
        message(WARNING "Could not parse Cargo version from string: ${CARGO_VERSION_RAW}")
        set(CARGO_VERSION "UNKNOWN" PARENT_SCOPE)
    endif()
endfunction()
