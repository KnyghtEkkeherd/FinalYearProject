#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "diffdrive_arduino::diffdrive_arduino" for configuration ""
set_property(TARGET diffdrive_arduino::diffdrive_arduino APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(diffdrive_arduino::diffdrive_arduino PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdiffdrive_arduino.so"
  IMPORTED_SONAME_NOCONFIG "libdiffdrive_arduino.so"
  )

list(APPEND _cmake_import_check_targets diffdrive_arduino::diffdrive_arduino )
list(APPEND _cmake_import_check_files_for_diffdrive_arduino::diffdrive_arduino "${_IMPORT_PREFIX}/lib/libdiffdrive_arduino.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
