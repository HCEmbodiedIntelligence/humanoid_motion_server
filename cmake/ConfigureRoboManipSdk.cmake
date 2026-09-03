# Configure the bundled, immutable robo_manip binary SDK snapshot.
#
# This module deliberately keeps all ABI-sensitive dependency resolution and
# linker policy next to the server that owns the SDK.  Consumers should link
# the imported targets below instead of using vendor paths directly.

if(DEFINED ENV{HUMANOID_MOTION_SDK_DEPS_PREFIX} AND
   NOT "$ENV{HUMANOID_MOTION_SDK_DEPS_PREFIX}" STREQUAL "")
  set(_default_sdk_deps_prefix "$ENV{HUMANOID_MOTION_SDK_DEPS_PREFIX}")
else()
  set(_default_sdk_deps_prefix "/opt/humanoid_motion_server/sdk-deps")
endif()
set(
  HUMANOID_MOTION_SDK_DEPS_PREFIX
  "${_default_sdk_deps_prefix}"
  CACHE PATH "Prefix containing the pinned open-source SDK runtime dependencies"
)
unset(_default_sdk_deps_prefix)

if(NOT IS_DIRECTORY "${HUMANOID_MOTION_SDK_DEPS_PREFIX}")
  message(FATAL_ERROR
    "Pinned robo_manip SDK dependencies were not found at "
    "${HUMANOID_MOTION_SDK_DEPS_PREFIX}. Set "
    "HUMANOID_MOTION_SDK_DEPS_PREFIX to the consolidated dependency prefix, "
    "or run scripts/install_sdk_dependencies_ubuntu2204.sh from the "
    "humanoid_motion_server source repository."
  )
endif()

set(HUMANOID_MOTION_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/robo_manip")

# Resolve exactly the ABI-compatible dependency set without permanently
# changing the prefix search order for the rest of the server build.
set(_sdk_saved_prefix_path "${CMAKE_PREFIX_PATH}")
list(PREPEND CMAKE_PREFIX_PATH "${HUMANOID_MOTION_SDK_DEPS_PREFIX}")
macro(_find_pinned_sdk_dependency package_name package_version)
  find_package(${package_name} ${package_version} EXACT QUIET CONFIG)
  if(NOT ${package_name}_FOUND)
    message(FATAL_ERROR
      "Missing pinned robo_manip SDK dependency: ${package_name} "
      "${package_version}. Expected its CMake package below "
      "${HUMANOID_MOTION_SDK_DEPS_PREFIX}. Run "
      "scripts/install_sdk_dependencies_ubuntu2204.sh, then clear the "
      "humanoid_motion_server CMake cache and rebuild."
    )
  endif()
endmacro()

_find_pinned_sdk_dependency(ruckig 0.17.3)
_find_pinned_sdk_dependency(toppra 0.6.8)
_find_pinned_sdk_dependency(NLopt 2.10.1)
_find_pinned_sdk_dependency(trac_ik_lib 0.1.0)
_find_pinned_sdk_dependency(eiquadprog 1.3.2)
_find_pinned_sdk_dependency(hpp-fcl 2.4.4)
_find_pinned_sdk_dependency(pinocchio 3.9.0)
set(CMAKE_PREFIX_PATH "${_sdk_saved_prefix_path}")
unset(_sdk_saved_prefix_path)

set(_sdk_libraries
  librobo_manip.so
  libmotion_control.so
  liblibplaco.so
)
foreach(sdk_library IN LISTS _sdk_libraries)
  if(NOT EXISTS "${HUMANOID_MOTION_SDK_ROOT}/lib/${sdk_library}")
    message(FATAL_ERROR "Missing bundled robo_manip SDK library: ${sdk_library}")
  endif()
endforeach()
unset(_sdk_libraries)

find_program(SHA256SUM_EXECUTABLE sha256sum)
if(NOT SHA256SUM_EXECUTABLE)
  message(FATAL_ERROR "sha256sum is required to verify the robo_manip SDK snapshot")
endif()

# Refuse to configure or build against a modified SDK snapshot.  The ALL target
# repeats the check so changes made after CMake configuration are also caught.
execute_process(
  COMMAND "${SHA256SUM_EXECUTABLE}" --check SHA256SUMS
  WORKING_DIRECTORY "${HUMANOID_MOTION_SDK_ROOT}"
  RESULT_VARIABLE _sdk_sha256_result
  OUTPUT_VARIABLE _sdk_sha256_output
  ERROR_VARIABLE _sdk_sha256_error
)
if(NOT _sdk_sha256_result EQUAL 0)
  message(FATAL_ERROR
    "robo_manip SDK SHA256 verification failed:\n"
    "${_sdk_sha256_output}${_sdk_sha256_error}"
  )
endif()
unset(_sdk_sha256_result)
unset(_sdk_sha256_output)
unset(_sdk_sha256_error)

add_custom_target(verify_robo_manip_sdk_sha256 ALL
  COMMAND "${SHA256SUM_EXECUTABLE}" --check SHA256SUMS
  WORKING_DIRECTORY "${HUMANOID_MOTION_SDK_ROOT}"
  COMMENT "Verifying the bundled robo_manip SDK SHA256 manifest"
  VERBATIM
)

set(HUMANOID_MOTION_SDK_BUILD_RPATH
  "${HUMANOID_MOTION_SDK_ROOT}/lib;${HUMANOID_MOTION_SDK_DEPS_PREFIX}/${CMAKE_INSTALL_LIBDIR};/opt/ros/humble/lib;/opt/ros/humble/lib/x86_64-linux-gnu"
)
set(HUMANOID_MOTION_SDK_INSTALL_RPATH
  "\$ORIGIN;${HUMANOID_MOTION_SDK_DEPS_PREFIX}/${CMAKE_INSTALL_LIBDIR};/opt/ros/humble/lib;/opt/ros/humble/lib/x86_64-linux-gnu"
)

function(_add_robo_manip_imported_target target_name library_name)
  add_library("humanoid_motion_server_sdk::${target_name}" SHARED IMPORTED GLOBAL)
  set_target_properties("humanoid_motion_server_sdk::${target_name}" PROPERTIES
    IMPORTED_LOCATION "${HUMANOID_MOTION_SDK_ROOT}/lib/${library_name}"
    IMPORTED_SONAME "${library_name}"
    INTERFACE_INCLUDE_DIRECTORIES
      "${HUMANOID_MOTION_SDK_ROOT}/include;${HUMANOID_MOTION_SDK_ROOT}/include/motion_control"
    BUILD_RPATH "${HUMANOID_MOTION_SDK_BUILD_RPATH}"
    INSTALL_RPATH "${HUMANOID_MOTION_SDK_INSTALL_RPATH}"
    INTERFACE_LINK_OPTIONS "LINKER:--disable-new-dtags"
  )
endfunction()

_add_robo_manip_imported_target(placo liblibplaco.so)
_add_robo_manip_imported_target(motion_control libmotion_control.so)
_add_robo_manip_imported_target(robo_manip librobo_manip.so)

set_property(TARGET humanoid_motion_server_sdk::placo PROPERTY
  INTERFACE_LINK_LIBRARIES
  "eiquadprog::eiquadprog;pinocchio::pinocchio_default;pinocchio::pinocchio_parsers;hpp-fcl::hpp-fcl;Eigen3::Eigen")
set_property(TARGET humanoid_motion_server_sdk::motion_control PROPERTY
  INTERFACE_LINK_LIBRARIES
  "humanoid_motion_server_sdk::placo;ruckig::ruckig;toppra::toppra;trac_ik_lib::trac_ik_lib;pinocchio::pinocchio_default;pinocchio::pinocchio_parsers;Eigen3::Eigen")
set_property(TARGET humanoid_motion_server_sdk::robo_manip PROPERTY
  INTERFACE_LINK_LIBRARIES
  "humanoid_motion_server_sdk::motion_control;Eigen3::Eigen")

# Preserve the SDK's immediate runtime closure as direct DT_NEEDED entries.
# Its immutable supplier RUNPATHs point at another machine, so normal
# --as-needed behavior would otherwise discard required transitive libraries.
set_property(TARGET humanoid_motion_server_sdk::placo APPEND PROPERTY
  INTERFACE_LINK_OPTIONS
  "LINKER:SHELL:--push-state --no-as-needed $<TARGET_FILE:eiquadprog::eiquadprog> $<TARGET_FILE:pinocchio::pinocchio_default> $<TARGET_FILE:pinocchio::pinocchio_parsers> $<TARGET_FILE:hpp-fcl::hpp-fcl> --pop-state")
set_property(TARGET humanoid_motion_server_sdk::motion_control APPEND PROPERTY
  INTERFACE_LINK_OPTIONS
  "LINKER:SHELL:--push-state --no-as-needed $<TARGET_FILE:humanoid_motion_server_sdk::placo> $<TARGET_FILE:ruckig::ruckig> $<TARGET_FILE:toppra::toppra> $<TARGET_FILE:trac_ik_lib::trac_ik_lib> $<TARGET_FILE:pinocchio::pinocchio_default> $<TARGET_FILE:pinocchio::pinocchio_parsers> --pop-state")
set_property(TARGET humanoid_motion_server_sdk::robo_manip APPEND PROPERTY
  INTERFACE_LINK_OPTIONS
  "LINKER:SHELL:--push-state --no-as-needed $<TARGET_FILE:humanoid_motion_server_sdk::motion_control> --pop-state")
