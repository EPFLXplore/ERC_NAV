# Find the LibSerial library and headers
find_path(LibSerial_INCLUDE_DIR SerialStream.h PATH_SUFFIXES libserial)
find_library(LibSerial_LIBRARY NAMES serial libserial PATH_SUFFIXES lib)

# Check if both include directory and library are found
if (LibSerial_INCLUDE_DIR AND LibSerial_LIBRARY)
    set(LibSerial_FOUND TRUE)
    set(LibSerial_INCLUDE_DIRS ${LibSerial_INCLUDE_DIR})
    set(LibSerial_LIBRARIES ${LibSerial_LIBRARY})
else()
    set(LibSerial_FOUND FALSE)
endif()

# Handle failure to find LibSerial
if (NOT LibSerial_FOUND)
    message(FATAL_ERROR "Could not find LibSerial library")
endif()
