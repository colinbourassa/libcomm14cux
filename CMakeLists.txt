cmake_minimum_required (VERSION 2.8)
project (libcomm14cux)

set(PROJECT_NAME "libcomm14cux")
set(PROJECT_DESCRIPTION "Library supporting serial communication with the Lucas 14CUX engine control module")
set(PROJECT_AUTHOR "Colin Bourassa <colin.bourassa@gmail.com>")
set(PROJECT_URL "https://github.com/colinbourassa/libcomm14cux")

set (COMM14CUX_VER_MAJOR 2)
set (COMM14CUX_VER_MINOR 1)
set (COMM14CUX_VER_PATCH 1)
set (COMM14CUX_VERSION "${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}")

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "bin")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "bin")
set (SOURCE_SUBDIR "${CMAKE_SOURCE_DIR}/src")

if ((CMAKE_SYSTEM_NAME MATCHES "OpenBSD") OR
    (CMAKE_SYSTEM_NAME MATCHES "FreeBSD"))
  set (CMAKE_INSTALL_PREFIX "/usr/local")
  message (STATUS "System is OpenBSD/FreeBSD/OS X; install path prefix is set to ${CMAKE_INSTALL_PREFIX}.")
elseif(CMAKE_SYSTEM_NAME MATCHES "NetBSD")
  set (CMAKE_INSTALL_PREFIX "/usr/pkg")
  message (STATUS "System is NetBSD; install path prefix is set to ${CMAKE_INSTALL_PREFIX}.")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set (CMAKE_INSTALL_PREFIX "/usr")
  message (STATUS "Defaulting install path prefix to ${CMAKE_INSTALL_PREFIX}.")
endif()

set(INSTALL_BIN_DIR "bin" CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR "lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "include" CACHE PATH "Installation directory for headers")
set(INSTALL_MAN_DIR "share/man" CACHE PATH "Installation directory for manual pages")
set(INSTALL_PKGCONFIG_DIR "lib/pkgconfig" CACHE PATH "Installation directory for pkgconfig (.pc) files")

set (CPACK_SOURCE_GENERATOR "TGZ")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VERSION}")
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

option (ENABLE_DOC_INSTALL "Enables installation of documentation (README, LICENSE, manpage) to the appropriate locations" ON)
option (ENABLE_TESTAPP_INSTALL "Enables installation of the read14cux/write14cux utilities" ON)

configure_file (
  "${SOURCE_SUBDIR}/comm14cux_version.h.in"
  "${CMAKE_BINARY_DIR}/comm14cux_version.h"
  )

set (LIBCOMM14CUX_PC "${CMAKE_BINARY_DIR}/libcomm14cux.pc")
configure_file ("${CMAKE_SOURCE_DIR}/libcomm14cux.pc.cmakein" ${LIBCOMM14CUX_PC} @ONLY)

include_directories (
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_BINARY_DIR}"
  "${CMAKE_SOURCE_DIR}/src"
  )

if (BUILD_STATIC STREQUAL "ON")
  add_library (comm14cux STATIC ${SOURCE_SUBDIR}/setup.c
                                ${SOURCE_SUBDIR}/protocol.c
                                ${SOURCE_SUBDIR}/data.c)
  set (LIBNAME "${PROJECT_NAME}.a")
  set (LIB_DESTINATION_DIR "${INSTALL_LIB_DIR}")
else()
  add_library (comm14cux SHARED ${SOURCE_SUBDIR}/setup.c
                                ${SOURCE_SUBDIR}/protocol.c
                                ${SOURCE_SUBDIR}/data.c)
  if (MINGW)
    set (LIBNAME "${PROJECT_NAME}.dll")
    set (LIB_DESTINATION_DIR "${INSTALL_BIN_DIR}")
    set (IMPORTLIBNAME "${PROJECT_NAME}.dll.a")
  else()
    set (LIBNAME "${PROJECT_NAME}.so")
    set (LIB_DESTINATION_DIR "${INSTALL_LIB_DIR}")
  endif()
endif()

target_link_libraries (comm14cux m)
add_executable (read14cux ${SOURCE_SUBDIR}/read14cux.c)
add_executable (write14cux ${SOURCE_SUBDIR}/write14cux.c)

set (BINDIR "${CMAKE_BINARY_DIR}/${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

if (MINGW)
  message (STATUS "Found MinGW platform.")

  # statically link against the C MinGW lib to avoid incurring an additional DLL dependency
  set (CMAKE_SHARED_LINKER_FLAGS "-static-libgcc")
  set (CMAKE_EXE_LINKER_FLAGS "-static-libgcc")

  # convert proper Unix-style newline characters into Windows-style
  configure_file ("${CMAKE_SOURCE_DIR}/README" "${CMAKE_BINARY_DIR}/README.TXT" NEWLINE_STYLE WIN32)
  configure_file ("${CMAKE_SOURCE_DIR}/LICENSE" "${CMAKE_BINARY_DIR}/LICENSE.TXT" NEWLINE_STYLE WIN32)

  set_target_properties (
    comm14cux
      PROPERTIES
        SOVERSION ${COMM14CUX_VER_MAJOR}
        VERSION ${COMM14CUX_VER_MAJOR}
  )

  target_link_libraries (read14cux comm14cux)
  target_link_libraries (write14cux comm14cux)

  install (FILES "${SOURCE_SUBDIR}/comm14cux.h"
                 "${CMAKE_BINARY_DIR}/comm14cux_version.h"
           DESTINATION "${INSTALL_INC_DIR}")
  install (FILES "${BINDIR}/${LIBNAME}"
           DESTINATION "${LIB_DESTINATION_DIR}")
  install (FILES "${LIBCOMM14CUX_PC}"
           DESTINATION "${INSTALL_PKGCONFIG_DIR}")

  if (NOT BUILD_STATIC STREQUAL "ON")
    install (FILES "${BINDIR}/${IMPORTLIBNAME}"
             DESTINATION "${INSTALL_LIB_DIR}")
  endif()

  # MXE requires that documentation and test applications not be installed
  # by default, so we provide a couple env vars to control this
  if (ENABLE_DOC_INSTALL)
    message (STATUS "Install will include documentation.")
    install (FILES "${CMAKE_BINARY_DIR}/README.TXT"
                   "${CMAKE_BINARY_DIR}/LICENSE.TXT"
             DESTINATION "doc")
  else()
    message (STATUS "Skipping installation of documentation.")
  endif()

  if (ENABLE_TESTAPP_INSTALL)
    message (STATUS "Install will include test applications.")
    install (PROGRAMS "${BINDIR}/read14cux.exe"
                      "${BINDIR}/write14cux.exe"
             DESTINATION "${INSTALL_BIN_DIR}")
  else()
    message (STATUS "Skipping installation of test applications.")
  endif()

  set (CPACK_GENERATOR "ZIP")
  set (CPACK_PACKAGE_NAME ${PROJECT_NAME})
  set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
  if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "")
    set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VERSION}-${CMAKE_SYSTEM_NAME}")
  else()
    set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
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
    set (MANNAME "${PROJECT_NAME}.3")
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

  set_target_properties (
    comm14cux
      PROPERTIES
        SOVERSION ${COMM14CUX_VER_MAJOR}
        VERSION   ${COMM14CUX_VERSION}
  )

  target_link_libraries (read14cux comm14cux pthread)
  target_link_libraries (write14cux comm14cux pthread)

  # set the installation destinations for the header files,
  # shared library binaries, and reference utility
  install (FILES "${SOURCE_SUBDIR}/comm14cux.h"
                 "${CMAKE_BINARY_DIR}/comm14cux_version.h"
           DESTINATION "${INSTALL_INC_DIR}"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

  install (FILES "${LIBCOMM14CUX_PC}"
           DESTINATION "${INSTALL_PKGCONFIG_DIR}"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

  if (BUILD_STATIC STREQUAL "ON")
    message (FATAL_ERROR "Static Linux/BSD builds not currently supported")
  else()
    if (CMAKE_SYSTEM_NAME MATCHES "OpenBSD")
      message (STATUS "Using OpenBSD-style shared library naming.")
      install (FILES "${BINDIR}/${PROJECT_NAME}.so"
                     "${BINDIR}/${PROJECT_NAME}.so.${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}"
               DESTINATION "${LIB_DESTINATION_DIR}"
               PERMISSIONS
                OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
    elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
      message (STATUS "Using Mac OS X-style shared library naming.")
      install (FILES "${BINDIR}/${PROJECT_NAME}.dylib"
                     "${BINDIR}/${PROJECT_NAME}.${COMM14CUX_VER_MAJOR}.dylib"
                     "${BINDIR}/${PROJECT_NAME}.${COMM14CUX_VERSION}.dylib"
               DESTINATION "lib"
               DESTINATION "${LIB_DESTINATION_DIR}"
               PERMISSIONS
                OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
    else()
      message (STATUS "Using Linux/FreeBSD-style shared library naming.")
      install (FILES "${BINDIR}/${PROJECT_NAME}.so"
                     "${BINDIR}/${PROJECT_NAME}.so.${COMM14CUX_VER_MAJOR}"
                     "${BINDIR}/${PROJECT_NAME}.so.${COMM14CUX_VERSION}"
               DESTINATION "${LIB_DESTINATION_DIR}"
               PERMISSIONS
                OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
    endif()
  endif()

  if (ENABLE_DOC_INSTALL)
    install (DIRECTORY DESTINATION "share/doc/${PROJECT_NAME}" DIRECTORY_PERMISSIONS
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
             DESTINATION "share/doc/${PROJECT_NAME}"
             PERMISSIONS
              OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
  endif()

  if (ENABLE_TESTAPP_INSTALL)
    install (PROGRAMS "${BINDIR}/read14cux"
             DESTINATION "${INSTALL_BIN_DIR}"
             PERMISSIONS
              OWNER_READ OWNER_EXECUTE OWNER_WRITE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)

    install (PROGRAMS "${BINDIR}/write14cux"
             DESTINATION "${INSTALL_BIN_DIR}"
             PERMISSIONS
              OWNER_READ OWNER_EXECUTE OWNER_WRITE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)
  endif()

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
    set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VERSION}-${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
    set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
    set (CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README")
    include(CPack)
  endif()

endif()

