if (ARDUINO)
    message (STATUS "Building for Arduino!")
    set (CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)
    set (ARDUINO_DEFAULT_BOARD uno)
    set (ARDUINO_DEFAULT_PORT /dev/ttyACM0)
    set (ARDUINO_DEFAULT_PROGRAMMER avrisp)
endif()

cmake_minimum_required (VERSION 2.8)
project (libcomm14cux)

set (COMM14CUX_VER_MAJOR 0)
set (COMM14CUX_VER_MINOR 5)
set (COMM14CUX_VER_PATCH 1)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set (SOURCE_SUBDIR "src")

set (CPACK_SOURCE_GENERATOR "TGZ" "ZIP")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}-src")
set (CPACK_SOURCE_IGNORE_FILES
      ".*deb$"
      ".*zip$"
      ".*gz$"
      ".*~$"
      ".*.dll.a$"
      "README.TXT"
      "LICENSE.TXT"
      ".git"
      "^CPack.*"
      "_CPack_Packages"
      "CPackConfig.cmake"
      "CPackSourceConfig.cmake"
      "CMakeFiles"
      "CMakeCache.txt"
      "cmake_install.cmake"
      "install_manifest.txt"
      "Makefile"
      "bin/lib.*"
      "bin/read14cux"
      "src/comm14cux_version.h$")

configure_file (
  "${SOURCE_SUBDIR}/comm14cux_version.h.in"
  "${SOURCE_SUBDIR}/comm14cux_version.h"
  )

add_library (comm14cux SHARED ${SOURCE_SUBDIR}/comm14cux.cpp)
add_executable (read14cux ${SOURCE_SUBDIR}/read14cux.cpp)

if (ARDUINO)

  print_board_list()
  print_programmer_list()
  set (comm14cux_lib_SRCS ${SOURCE_SUBDIR}/comm14cux.cpp)
  set (comm14cux_lib_HDRS ${SOURCE_SUBDIR}/comm14cux.h)
  generate_arduino_library(comm14cux_lib)

elseif (MINGW)
  message (STATUS "Found Windows/MinGW platform.")

  find_library (MINGW_LIBGCC libgcc_s_dw2-1)
  if (MINGW_LIBGCC)
    message (STATUS "Found MinGW's libgcc is at ${MINGW_LIBGCC}")
  else ()
    message (SEND_ERROR "Could not find MinGW's libgcc_s_dw2-1.dll!")
  endif ()

  # convert Unix-style newline characters into Windows-style
  file (READ "README" README_CONTENTS)
  file (READ "LICENSE" LICENSE_CONTENTS)
  file (WRITE "README.TXT" ${README_CONTENTS})
  file (WRITE "LICENSE.TXT" ${LICENSE_CONTENTS})

  set (CMAKE_INSTALL_PREFIX "C:/comm14cux")

  set_target_properties (
    comm14cux
      PROPERTIES
        SOVERSION ${COMM14CUX_VER_MAJOR}
        VERSION ${COMM14CUX_VER_MAJOR}
  )

  target_link_libraries (read14cux comm14cux)

  install (FILES ${SOURCE_SUBDIR}/comm14cux.h
                 ${SOURCE_SUBDIR}/comm14cux_version.h
           DESTINATION "include")
  install (FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libcomm14cux.dll
           DESTINATION ".")
  install (FILES "README.txt"
                 "LICENSE.txt"
           DESTINATION "doc")
  install (PROGRAMS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/read14cux.exe
           DESTINATION ".")
  install (PROGRAMS ${MINGW_LIBGCC}
	   DESTINATION ".")

  set (CPACK_GENERATOR "ZIP")
  set (CPACK_PACKAGE_NAME ${PROJECT_NAME})
  set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
  set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}-${CMAKE_SYSTEM_NAME}")
  include (CPack)

else()
  message (STATUS "Defaulting to Linux build configuration.")

  set (CMAKE_SKIP_RPATH TRUE)
  set (CMAKE_INSTALL_PREFIX "/usr")

  set_target_properties (
    comm14cux
      PROPERTIES
        SOVERSION ${COMM14CUX_VER_MAJOR}
        VERSION ${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}
  )
  target_link_libraries (read14cux comm14cux)

  # set the installation destinations for the header files,
  # shared library binaries, and reference utility
  install (FILES ${SOURCE_SUBDIR}/comm14cux.h
                 ${SOURCE_SUBDIR}/comm14cux_version.h
           DESTINATION "include"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

  install (FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libcomm14cux.so
                 ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libcomm14cux.so.${COMM14CUX_VER_MAJOR}
                 ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libcomm14cux.so.${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}
           DESTINATION "lib"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

  install (DIRECTORY DESTINATION "share/doc/comm14cux" DIRECTORY_PERMISSIONS
            OWNER_READ OWNER_EXECUTE OWNER_WRITE
            GROUP_READ GROUP_EXECUTE
            WORLD_READ WORLD_EXECUTE)

  install (FILES "README"
                 "LICENSE"
           DESTINATION "share/doc/comm14cux"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

  install (PROGRAMS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/read14cux
           DESTINATION "bin"
           PERMISSIONS
            OWNER_READ OWNER_EXECUTE OWNER_WRITE
            GROUP_READ GROUP_EXECUTE
            WORLD_READ WORLD_EXECUTE)

  # set the architecture flag that will be used by the Debian package builder
  if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    set (CPACK_DEBIAN_PACKAGE_ARCHITECTURE amd64)
  elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "i686")
    set (CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
  endif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")

  set (CPACK_GENERATOR "DEB")
  set (CPACK_PACKAGE_NAME ${PROJECT_NAME})
  set (CPACK_PACKAGE_VERSION_MAJOR ${COMM14CUX_VER_MAJOR})
  set (CPACK_PACKAGE_VERSION_MINOR ${COMM14CUX_VER_MINOR})
  set (CPACK_PACKAGE_VERSION_PATCH ${COMM14CUX_VER_PATCH})
  set (CPACK_DEBIAN_PACKAGE_NAME ${CPACK_PACKAGE_NAME})
  set (CPACK_DEBIAN_PACKAGE_MAINTAINER "Colin Bourassa <lucas.14cux@gmail.com>")
  set (CPACK_DEBIAN_PACKAGE_SECTION "Libraries")
  set (CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.13), libstdc++6 (>= 4.6.1)")
  set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "Library supporting serial communication with the Lucas 14CUX engine control module")
  set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}-${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
  set (CPACK_RESOURCE_FILE_LICENSE "LICENSE")
  set (CPACK_RESOURCE_FILE_README "README")

  include(CPack)

endif()

