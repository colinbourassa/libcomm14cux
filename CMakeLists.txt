cmake_minimum_required (VERSION 2.8)
project (libcomm14cux)

set(PROJECT_NAME "libcomm14cux")
set(PROJECT_DESCRIPTION "Library supporting serial communication with the Lucas 14CUX engine control module")
set(PROJECT_AUTHOR "Colin Bourassa <colin.bourassa@gmail.com>")
set(PROJECT_URL "https://github.com/colinbourassa/libcomm14cux")

set (COMM14CUX_VER_MAJOR 2)
set (COMM14CUX_VER_MINOR 0)
set (COMM14CUX_VER_PATCH 0)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set (SOURCE_SUBDIR "${CMAKE_SOURCE_DIR}/src")

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
      "bin/write14cux"
      "src/comm14cux_version.h$")

configure_file (
  "${SOURCE_SUBDIR}/comm14cux_version.h.in"
  "${SOURCE_SUBDIR}/comm14cux_version.h"
  )

include_directories (
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/src"
  "${CMAKE_BINARY_DIR}/src"
  )

add_library (comm14cux SHARED ${SOURCE_SUBDIR}/setup.c
                              ${SOURCE_SUBDIR}/protocol.c
                              ${SOURCE_SUBDIR}/data.c)
target_link_libraries (comm14cux m)
add_executable (read14cux ${SOURCE_SUBDIR}/read14cux.c)
add_executable (write14cux ${SOURCE_SUBDIR}/write14cux.c)

if (MINGW)
  message (STATUS "Found Windows/MinGW platform.")

  # statically link against the C MinGW lib to avoid incurring an additional DLL dependency
  set (CMAKE_SHARED_LINKER_FLAGS "-static-libgcc")
  set (CMAKE_EXE_LINKER_FLAGS "-static-libgcc")

  set (BINDIR "${CMAKE_BINARY_DIR}/${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

  # convert proper Unix-style newline characters into Windows-style
  configure_file ("${CMAKE_SOURCE_DIR}/README" "${CMAKE_BINARY_DIR}/README.TXT" NEWLINE_STYLE WIN32)
  configure_file ("${CMAKE_SOURCE_DIR}/LICENSE" "${CMAKE_BINARY_DIR}/LICENSE.TXT" NEWLINE_STYLE WIN32)

  if ("$ENV{SystemDrive}" STREQUAL "")
    message (STATUS "Windows system drive not found; creating 'install' directory under current directory...")
    set (CMAKE_INSTALL_PREFIX "./install")
  else()
    message (STATUS "Defaulting installation path to: $ENV{SystemDrive}/comm14cux")
    set (CMAKE_INSTALL_PREFIX "$ENV{SystemDrive}/comm14cux")
  endif()

  set_target_properties (
    comm14cux
      PROPERTIES
        SOVERSION ${COMM14CUX_VER_MAJOR}
        VERSION ${COMM14CUX_VER_MAJOR}
  )

  target_link_libraries (read14cux comm14cux)
  target_link_libraries (write14cux comm14cux)

  install (FILES ${SOURCE_SUBDIR}/comm14cux.h
           DESTINATION "include")
  install (FILES ${BINDIR}/libcomm14cux.dll
           DESTINATION ".")
  install (FILES "${CMAKE_BINARY_DIR}/README.txt"
                 "${CMAKE_BINARY_DIR}/LICENSE.txt"
           DESTINATION "doc")
  install (PROGRAMS ${BINDIR}/read14cux.exe
           DESTINATION ".")
  install (PROGRAMS ${BINDIR}/write14cux.exe
           DESTINATION ".")

  set (CPACK_GENERATOR "ZIP")
  set (CPACK_PACKAGE_NAME ${PROJECT_NAME})
  set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
  if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "")
    set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}-${CMAKE_SYSTEM_NAME}")
  else()
    set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
  endif()
  include (CPack)

else()

  message (STATUS "Defaulting to UNIX/Linux/BSD build configuration.")

  find_program (GZIP_TOOL
                NAMES gzip
                PATHS /bin
                      /usr/bin
                      /usr/local/bin)

  if (GZIP_TOOL)
    set (MANNAME "libcomm14cux.3")
    set (MAN "${CMAKE_SOURCE_DIR}/${MANNAME}")
    set (MAN-GZ "${CMAKE_BINARY_DIR}/${MANNAME}.gz")
    add_custom_command (OUTPUT ${MAN-GZ}
                        COMMAND ${GZIP_TOOL} -c ${MAN} > ${MAN-GZ}
                        DEPENDS ${MAN}
                        COMMENT "Building ${MAN-GZ}..."
                        VERBATIM)
    add_custom_target (man ALL
                       DEPENDS ${MAN-GZ})
  else ()
    message (WARNING "gzip not found! Manual page will not be installed.")
  endif (GZIP_TOOL)

  set (CMAKE_SKIP_RPATH TRUE)

  if ((CMAKE_SYSTEM_NAME MATCHES "OpenBSD") OR
      (CMAKE_SYSTEM_NAME MATCHES "FreeBSD") OR
      (CMAKE_SYSTEM_NAME MATCHES "Darwin"))
    set (CMAKE_INSTALL_PREFIX "/usr/local")
    message (STATUS "System is OpenBSD/FreeBSD/OS X; install path prefix is set to ${CMAKE_INSTALL_PREFIX}.")
  elseif(CMAKE_SYSTEM_NAME MATCHES "NetBSD")
    set (CMAKE_INSTALL_PREFIX "/usr/pkg")
    message (STATUS "System is NetBSD; install path prefix is set to ${CMAKE_INSTALL_PREFIX}.")
  else()
    set (CMAKE_INSTALL_PREFIX "/usr")
    message (STATUS "Install path prefix is set to ${CMAKE_INSTALL_PREFIX}.")
  endif()

  set_target_properties (
    comm14cux
      PROPERTIES
        SOVERSION ${COMM14CUX_VER_MAJOR}
        VERSION   ${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}
  )

  set (BINDIR "${CMAKE_BINARY_DIR}/${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

  target_link_libraries (read14cux comm14cux pthread)
  target_link_libraries (write14cux comm14cux pthread)

  # set the installation destinations for the header files,
  # shared library binaries, and reference utility
  install (FILES ${SOURCE_SUBDIR}/comm14cux.h
           DESTINATION "include"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

  if (CMAKE_SYSTEM_NAME MATCHES "OpenBSD")
    message (STATUS "Using OpenBSD-style shared library naming.")
    install (FILES ${BINDIR}/libcomm14cux.so
                   ${BINDIR}/libcomm14cux.so.${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}
             DESTINATION "lib"
             PERMISSIONS
              OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
  elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    message (STATUS "Using Mac OS X-style shared library naming.")
    install (FILES ${BINDIR}/libcomm14cux.dylib
                   ${BINDIR}/libcomm14cux.${COMM14CUX_VER_MAJOR}.dylib
                   ${BINDIR}/libcomm14cux.${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}.dylib
             DESTINATION "lib"
             PERMISSIONS
              OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
  else()
    message (STATUS "Using Linux/FreeBSD-style shared library naming.")
    install (FILES ${BINDIR}/libcomm14cux.so
                   ${BINDIR}/libcomm14cux.so.${COMM14CUX_VER_MAJOR}
                   ${BINDIR}/libcomm14cux.so.${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}
             DESTINATION "lib"
             PERMISSIONS
              OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
  endif()

  install (DIRECTORY DESTINATION "share/doc/comm14cux" DIRECTORY_PERMISSIONS
            OWNER_READ OWNER_EXECUTE OWNER_WRITE
            GROUP_READ GROUP_EXECUTE
            WORLD_READ WORLD_EXECUTE)

  if (GZIP_TOOL)
    install (FILES "${MAN-GZ}"
             DESTINATION "share/man/man3"
             PERMISSIONS
              OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
  endif (GZIP_TOOL)

  install (FILES "${CMAKE_SOURCE_DIR}/README"
                 "${CMAKE_SOURCE_DIR}/LICENSE"
           DESTINATION "share/doc/comm14cux"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

  install (PROGRAMS ${BINDIR}/read14cux
           DESTINATION "bin"
           PERMISSIONS
            OWNER_READ OWNER_EXECUTE OWNER_WRITE
            GROUP_READ GROUP_EXECUTE
            WORLD_READ WORLD_EXECUTE)

  install (PROGRAMS ${BINDIR}/write14cux
           DESTINATION "bin"
           PERMISSIONS
            OWNER_READ OWNER_EXECUTE OWNER_WRITE
            GROUP_READ GROUP_EXECUTE
            WORLD_READ WORLD_EXECUTE)

  if (CMAKE_SYSTEM_NAME MATCHES "Linux")
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
    set (CPACK_DEBIAN_PACKAGE_MAINTAINER "${PROJECT_AUTHOR}")
    set (CPACK_DEBIAN_PACKAGE_SECTION "Libraries")
    set (CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.13)")
    set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
    set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}-${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
    set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
    set (CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README")
    include(CPack)
  endif()

endif()

