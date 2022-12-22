#[=======================================================================[.rst:
FindArtecSDK
-----------

Find the Artec scanner SDK.

Hints
^^^^^

The following variables may be set to control search behavior:

``ARTECSDK_ROOT_DIR``
  Set to the root directory of an Artec SDK installation.

#]=======================================================================]

if(NOT MSVC)
    message(FATAL_ERROR "Artec SDK only avaliable on Windows MSVC")
endif()

set(ARTECSDK_VERSION 2.0) # Maybe? Version not in any files in the SDK...

find_path(ARTECSDK_INCLUDE_DIR
    NAMES
      artec/sdk/base/BaseSdkDefines.h
    HINTS
      ${ARTECSDK_ROOT_DIR}
    PATHS
      "C:\\Program Files\\Artec\\Artec 3D Scanning SDK"
    PATH_SUFFIXES
      include
)

message(STATUS "Found ARTECSDK_INCLUDE_DIR=${ARTECSDK_INCLUDE_DIR}")

macro(_FIND_ARTECSDK_LIBRARY _varname _targetname _libname )

find_library(${_varname}
  NAMES ${_libname}
  HINTS
    ${ARTECSDK_ROOT_DIR}
  PATHS
    "C:\\Program Files\\Artec\\Artec 3D Scanning SDK"
  PATH_SUFFIXES
    bin-x64

    )
message(STATUS "Found ${_varname}=${${_varname}}")

if(${_varname})
  if(NOT TARGET ArtecSDK::${_targetname})
    add_library(ArtecSDK::${_targetname} UNKNOWN IMPORTED)
    set_target_properties(ArtecSDK::${_targetname} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${ARTECSDK_INCLUDE_DIR}")
    if(${_varname})
      set_target_properties(ArtecSDK::${_targetname} PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${${_varname}}"
        )
    endif()
  endif()
endif()


endmacro()

_FIND_ARTECSDK_LIBRARY(ARTECSDK_BASE_LIBRARY Base artec-sdk-base)
_FIND_ARTECSDK_LIBRARY(ARTECSDK_ALGORITHMS_LIBRARY Algorithms artec-sdk-algorithms)
_FIND_ARTECSDK_LIBRARY(ARTECSDK_CAPTURING_LIBRARY Capturing artec-sdk-capturing)
_FIND_ARTECSDK_LIBRARY(ARTECSDK_PROJECT_LIBRARY Project artec-sdk-project)
_FIND_ARTECSDK_LIBRARY(ARTECSDK_SCANNING_LIBRARY Scanning artec-sdk-scanning)

unset(_FIND_ARTECSDK_LIBRARY)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ArtecSDK  REQUIRED_VARS  ARTECSDK_INCLUDE_DIR  
  ARTECSDK_BASE_LIBRARY ARTECSDK_ALGORITHMS_LIBRARY ARTECSDK_CAPTURING_LIBRARY
  ARTECSDK_PROJECT_LIBRARY ARTECSDK_SCANNING_LIBRARY
  VERSION_VAR ARTECSDK_VERSION )