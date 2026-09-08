# Select one coherent SDK dependency installation. EXACT versions alone are
# insufficient: ROS can ship the same version with different build options.
# This is the layout written by install_sdk_dependencies_ubuntu2204.sh.
set(_robo_manip_dependencies
  "ruckig|0.17.3|lib/cmake/ruckig|ruckig-config.cmake"
  "toppra|0.6.8|lib/cmake/toppra|toppraConfig.cmake"
  "NLopt|2.10.1|lib/cmake/nlopt|NLoptConfig.cmake"
  "trac_ik_lib|0.1.0|share/trac_ik_lib/cmake|trac_ik_libConfig.cmake"
  "eiquadprog|1.3.2|lib/cmake/eiquadprog|eiquadprogConfig.cmake"
  "octomap|1.9.8|share/octomap|octomap-config.cmake"
  "hpp-fcl|2.4.4|lib/cmake/hpp-fcl|hpp-fclConfig.cmake"
  "pinocchio|3.9.0|lib/cmake/pinocchio|pinocchioConfig.cmake")
set(_robo_manip_targets
  ruckig::ruckig toppra::toppra NLopt::nlopt trac_ik_lib::trac_ik_lib
  eiquadprog::eiquadprog octomap octomath hpp-fcl::hpp-fcl
  pinocchio::pinocchio_default pinocchio::pinocchio_parsers)

function(_robo_manip_check_target_origin target)
  if(NOT TARGET "${target}")
    return()
  endif()
  get_target_property(configurations "${target}" IMPORTED_CONFIGURATIONS)
  set(properties IMPORTED_LOCATION)
  foreach(configuration IN LISTS configurations)
    string(TOUPPER "${configuration}" configuration)
    list(APPEND properties "IMPORTED_LOCATION_${configuration}")
  endforeach()
  set(found_location FALSE)
  foreach(property IN LISTS properties)
    get_target_property(location "${target}" "${property}")
    if(location)
      set(found_location TRUE)
      get_filename_component(location "${location}" REALPATH)
      if(NOT location MATCHES "^/usr/local/lib/")
        message(FATAL_ERROR
          "Conflicting SDK target ${target}: ${location}. The pinned SDK "
          "dependencies must all come from /usr/local. Load the motion SDK "
          "before another Pinocchio/Ruckig stack, or isolate the two consumers.")
      endif()
    endif()
  endforeach()
  if(NOT found_location)
    message(FATAL_ERROR "Cannot verify the installed library for SDK target ${target}")
  endif()
endfunction()

# Already-created imported targets cannot safely be redirected by changing a
# *_DIR cache entry. Refuse that situation instead of silently mixing ABIs.
foreach(_target IN LISTS _robo_manip_targets)
  _robo_manip_check_target_origin("${_target}")
endforeach()

# Set every package location BEFORE reading any config, including transitive
# dependencies such as hpp-fcl -> octomap and trac_ik_lib -> NLopt. Repair old
# ROS *_DIR cache entries too; changing CMAKE_PREFIX_PATH alone does not do so.
foreach(_dependency IN LISTS _robo_manip_dependencies)
  string(REPLACE "|" ";" _fields "${_dependency}")
  list(GET _fields 0 _package)
  list(GET _fields 2 _directory)
  list(GET _fields 3 _config)
  set(_directory "/usr/local/${_directory}")
  if(NOT EXISTS "${_directory}/${_config}")
    message(FATAL_ERROR
      "Missing pinned SDK CMake package: ${_directory}/${_config}. Run "
      "scripts/install_sdk_dependencies_ubuntu2204.sh --jobs 2. "
      "A ROS copy is not a substitute for this SDK's system installation.")
  endif()
  set("${_package}_DIR" "${_directory}" CACHE PATH "Pinned robo_manip SDK dependency" FORCE)
  set("${_package}_DIR" "${_directory}")
endforeach()

foreach(_dependency IN LISTS _robo_manip_dependencies)
  string(REPLACE "|" ";" _fields "${_dependency}")
  list(GET _fields 0 _package)
  list(GET _fields 1 _version)
  find_package(${_package} ${_version} EXACT REQUIRED CONFIG
    PATHS "${${_package}_DIR}" NO_DEFAULT_PATH)
  message(STATUS "Pinned SDK ${_package} ${_version}: ${${_package}_DIR}")
endforeach()
foreach(_target IN LISTS _robo_manip_targets)
  if(NOT TARGET "${_target}")
    message(FATAL_ERROR "Pinned SDK dependency did not export ${_target}")
  endif()
  _robo_manip_check_target_origin("${_target}")
endforeach()

unset(_robo_manip_dependencies)
unset(_robo_manip_targets)
unset(_target)
unset(_dependency)
unset(_fields)
unset(_package)
unset(_version)
unset(_directory)
unset(_config)
