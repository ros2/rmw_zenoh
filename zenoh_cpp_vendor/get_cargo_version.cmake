# Detect the installed Cargo version and set CARGO_VERSION in the parent scope.
# Sets CARGO_VERSION to an empty string if cargo is not found or version cannot be parsed.
function(get_cargo_version)
  execute_process(
    COMMAND cargo --version
    OUTPUT_VARIABLE CARGO_VERSION_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE CARGO_RESULT
  )
  if(NOT CARGO_RESULT EQUAL 0)
    set(CARGO_VERSION "" PARENT_SCOPE)
    return()
  endif()
  string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" _version "${CARGO_VERSION_OUTPUT}")
  set(CARGO_VERSION "${_version}" PARENT_SCOPE)
endfunction()
