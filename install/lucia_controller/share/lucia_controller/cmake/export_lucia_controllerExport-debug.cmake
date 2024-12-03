#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lucia_controller::lucia_controller_node" for configuration "Debug"
set_property(TARGET lucia_controller::lucia_controller_node APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(lucia_controller::lucia_controller_node PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/lucia_controller/lucia_controller_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS lucia_controller::lucia_controller_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_lucia_controller::lucia_controller_node "${_IMPORT_PREFIX}/lib/lucia_controller/lucia_controller_node" )

# Import target "lucia_controller::lucia_controller_lib" for configuration "Debug"
set_property(TARGET lucia_controller::lucia_controller_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(lucia_controller::lucia_controller_lib PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/liblucia_controller_lib.so"
  IMPORTED_SONAME_DEBUG "liblucia_controller_lib.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lucia_controller::lucia_controller_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_lucia_controller::lucia_controller_lib "${_IMPORT_PREFIX}/lib/liblucia_controller_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
