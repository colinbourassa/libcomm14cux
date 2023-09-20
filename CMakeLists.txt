cmake_minimum_required (VERSION 3.21)
project (libcomm14cux LANGUAGES C)
include (GNUInstallDirs)

set(PROJECT_NAME "libcomm14cux")
set(PROJECT_DESCRIPTION "Library supporting serial communication with the Lucas 14CUX engine control module")
set(PROJECT_AUTHOR "Colin Bourassa <colin.bourassa@gmail.com>")
set(PROJECT_URL "https://github.com/colinbourassa/libcomm14cux")

set (COMM14CUX_VER_MAJOR 3)
set (COMM14CUX_VER_MINOR 0)
set (COMM14CUX_VER_PATCH 0)
set (COMM14CUX_VERSION "${COMM14CUX_VER_MAJOR}.${COMM14CUX_VER_MINOR}.${COMM14CUX_VER_PATCH}")

set (SOURCE_SUBDIR "${CMAKE_SOURCE_DIR}/src")

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
      "comm14cux_version.h$"
      "build/"
      "build-win32")

option (ENABLE_DOC_INSTALL "Enables installation of documentation (README, LICENSE, manpage) to the appropriate locations" OFF)
option (ENABLE_TESTAPP_INSTALL "Enables installation of the read14cux/write14cux utilities" OFF)
option (ENABLE_PKGCONFIG_INSTALL "Enables installation of a pkgconfig configuration file" ON)

find_package (LibFTDI1 CONFIG REQUIRED)
find_package (PkgConfig)
pkg_check_modules(LIBRARY_USB libusb-1.0 IMPORTED_TARGET)

if (NOT LIBRARY_USB_FOUND)
  message(FATAL_ERROR "LibUSB 1.0 required to build.")
endif()

configure_file ("${SOURCE_SUBDIR}/comm14cux_version.h.in"
                "${CMAKE_BINARY_DIR}/comm14cux_version.h")

include_directories ("${CMAKE_SOURCE_DIR}/src" "${CMAKE_BINARY_DIR}" "${LIBRARY_USB_INCLUDE_DIRS}")

if (BUILD_STATIC STREQUAL "ON")
  add_library (comm14cux STATIC ${SOURCE_SUBDIR}/setup.c
                                ${SOURCE_SUBDIR}/protocol.c
                                ${SOURCE_SUBDIR}/data.c)
else()
  add_library (comm14cux SHARED ${SOURCE_SUBDIR}/setup.c
                                ${SOURCE_SUBDIR}/protocol.c
                                ${SOURCE_SUBDIR}/data.c)
endif()

target_link_libraries (comm14cux m ftdi1 usb-1.0)

add_executable (read14cux ${SOURCE_SUBDIR}/read14cux.c)
add_executable (write14cux ${SOURCE_SUBDIR}/write14cux.c)

if (MINGW OR WIN32)

  message (STATUS "Found Win32/MinGW platform.")

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
        VERSION   ${COMM14CUX_VER_MAJOR}
  )

  target_link_libraries (read14cux comm14cux)
  target_link_libraries (write14cux comm14cux)

  # MXE requires that documentation and test applications not be installed
  # by default, so we provide a couple env vars to control this
  if (ENABLE_DOC_INSTALL)
    message (STATUS "Install will include documentation.")
    install (FILES "${CMAKE_BINARY_DIR}/README.TXT"
                   "${CMAKE_BINARY_DIR}/LICENSE.TXT"
             DESTINATION "${CMAKE_INSTALL_DOCDIR}")
  else()
    message (STATUS "Skipping installation of documentation.")
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

  if (ENABLE_DOC_INSTALL)
    install (DIRECTORY DESTINATION "${CMAKE_INSTALL_DOCDIR}" DIRECTORY_PERMISSIONS
              OWNER_READ OWNER_EXECUTE OWNER_WRITE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)

    if (GZIP_TOOL)
      install (FILES "${MAN-GZ}"
               DESTINATION "${CMAKE_INSTALL_MANDIR}/man3"
               PERMISSIONS
                OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
    endif (GZIP_TOOL)

    install (FILES "${CMAKE_SOURCE_DIR}/README"
                   "${CMAKE_SOURCE_DIR}/LICENSE"
             DESTINATION "${CMAKE_INSTALL_DOCDIR}"
             PERMISSIONS
              OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
  else()
    message (STATUS "Skipping installation of documentation.")
  endif()

  #
  # If building on Linux, set up to create a Debian package
  #
  if (CMAKE_SYSTEM_NAME MATCHES "Linux")

    find_program (DPKG_CMD dpkg)
    if (NOT DPKG_CMD)
      message (STATUS "'dpkg' not found in path; defaulting to i386.")
      set (CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
    endif()
    execute_process (COMMAND "${DPKG_CMD}" --print-architecture
                     OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
                     OUTPUT_STRIP_TRAILING_WHITESPACE)
    message (STATUS "CPack architecture for dpkg: ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

    set (CPACK_GENERATOR "DEB")
    set (CPACK_PACKAGE_NAME ${PROJECT_NAME})
    set (CPACK_PACKAGE_VERSION_MAJOR ${COMM14CUX_VER_MAJOR})
    set (CPACK_PACKAGE_VERSION_MINOR ${COMM14CUX_VER_MINOR})
    set (CPACK_PACKAGE_VERSION_PATCH ${COMM14CUX_VER_PATCH})
    set (CPACK_DEBIAN_PACKAGE_NAME ${CPACK_PACKAGE_NAME})
    set (CPACK_DEBIAN_PACKAGE_MAINTAINER "${PROJECT_AUTHOR}")
    set (CPACK_DEBIAN_PACKAGE_SECTION "Libraries")
    set (CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.13), libusb-1.0-0 (>= 1.0.22), libftdi1-2 (>= 1.5)")
    set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
    set (CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${COMM14CUX_VERSION}-${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
    set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
    set (CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README")
    include(CPack)
  endif()

endif()

#
# install the public headers
#
install (FILES "${SOURCE_SUBDIR}/comm14cux.h"
               "${CMAKE_BINARY_DIR}/comm14cux_version.h"
         DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
         PERMISSIONS
          OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

#
# install the library
#
install (TARGETS comm14cux
         LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")

#
# install any library dependencies
#
if (MINGW OR WIN32)

  # When we call find_library() below, we're only looking for DLLs, so set the suffix list appropriately
  set (CMAKE_FIND_LIBRARY_SUFFIXES ".dll")

  find_library (LIBRARY_USB libusb-1.0)
  if (LIBRARY_USB)
    message (STATUS "Found libusb1 at ${LIBRARY_USB}")
  else ()
    message (SEND_ERROR "Could not find libusb1!")
  endif ()

  find_library (LIBRARY_FTDI libftdi1)
  if (LIBRARY_FTDI)
    message (STATUS "Found libftdi1 at ${LIBRARY_FTDI}")
  else ()
    message (SEND_ERROR "Could not find libftdi1!")
  endif ()

  install (FILES "${LIBRARY_USB}" "${LIBRARY_FTDI}"
           DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

#
# install the pkgconfig file
#
if (ENABLE_PKGCONFIG_INSTALL)
  set (LIBCOMM14CUX_PC "${CMAKE_BINARY_DIR}/libcomm14cux.pc")
  configure_file ("${CMAKE_SOURCE_DIR}/libcomm14cux.pc.cmakein" ${LIBCOMM14CUX_PC} @ONLY)

  install (FILES "${LIBCOMM14CUX_PC}"
           DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
           PERMISSIONS
            OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
else()
  message (STATUS "Skipping installation of pkgconfig file.")
endif()

#
# install the test applications
#
if (ENABLE_TESTAPP_INSTALL)
  install (TARGETS read14cux write14cux
           RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
else()
  message (STATUS "Skipping installation of test applications (read14cux / write14cux).")
endif()

