#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "serial::serial_cmake" for configuration "RelWithDebInfo"
set_property(TARGET serial::serial_cmake APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(serial::serial_cmake PROPERTIES
  IMPORTED_IMPLIB_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/serial_cmake.lib"
  )

list(APPEND _cmake_import_check_targets serial::serial_cmake )
list(APPEND _cmake_import_check_files_for_serial::serial_cmake "${_IMPORT_PREFIX}/lib/serial_cmake.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
