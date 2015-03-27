# This generates a header file, from the named json file, containing a string literal
# named com_osvr_DeadReckoningOrientation_json (not null terminated)
# The file must be added as a source file to some target (as below) to be generated.
osvr_convert_json(com_osvr_DeadReckoningOrientation_json
    com_osvr_DeadReckoningOrientation.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_DeadReckoningOrientation_json.h")

# Be able to find our generated header file.
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# This is just a helper function wrapping CMake's add_library command that
# sets up include dirs, libraries, and naming convention (no leading "lib")
# for an OSVR plugin. It also installs the plugin into the right directory.
# Pass as many source files as you need. See osvrAddPlugin.cmake for full docs.
osvr_add_plugin(NAME com_osvr_DeadReckoningOrientation
    CPP # indicates we'd like to use the C++ wrapper
    SOURCES
    com_osvr_DeadReckoningOrientation.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_DeadReckoningOrientation_json.h")

# If you use other libraries, find them and add a line like:
# target_link_libraries(com_osvr_DeadReckoningOrientation AnyOtherLibraries)

set_target_properties(com_osvr_DeadReckoningOrientation PROPERTIES
    FOLDER "OSVR Plugins")

