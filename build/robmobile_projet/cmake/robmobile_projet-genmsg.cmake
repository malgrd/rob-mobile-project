# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robmobile_projet: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irobmobile_projet:/home/marion/catkin_ws/src/robmobile_projet/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robmobile_projet_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg" NAME_WE)
add_custom_target(_robmobile_projet_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robmobile_projet" "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robmobile_projet
  "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robmobile_projet
)

### Generating Services

### Generating Module File
_generate_module_cpp(robmobile_projet
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robmobile_projet
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robmobile_projet_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robmobile_projet_generate_messages robmobile_projet_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg" NAME_WE)
add_dependencies(robmobile_projet_generate_messages_cpp _robmobile_projet_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robmobile_projet_gencpp)
add_dependencies(robmobile_projet_gencpp robmobile_projet_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robmobile_projet_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robmobile_projet
  "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robmobile_projet
)

### Generating Services

### Generating Module File
_generate_module_lisp(robmobile_projet
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robmobile_projet
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robmobile_projet_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robmobile_projet_generate_messages robmobile_projet_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg" NAME_WE)
add_dependencies(robmobile_projet_generate_messages_lisp _robmobile_projet_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robmobile_projet_genlisp)
add_dependencies(robmobile_projet_genlisp robmobile_projet_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robmobile_projet_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robmobile_projet
  "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robmobile_projet
)

### Generating Services

### Generating Module File
_generate_module_py(robmobile_projet
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robmobile_projet
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robmobile_projet_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robmobile_projet_generate_messages robmobile_projet_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marion/catkin_ws/src/robmobile_projet/msg/Tab_point.msg" NAME_WE)
add_dependencies(robmobile_projet_generate_messages_py _robmobile_projet_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robmobile_projet_genpy)
add_dependencies(robmobile_projet_genpy robmobile_projet_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robmobile_projet_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robmobile_projet)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robmobile_projet
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robmobile_projet_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robmobile_projet_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robmobile_projet)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robmobile_projet
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robmobile_projet_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robmobile_projet_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robmobile_projet)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robmobile_projet\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robmobile_projet
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robmobile_projet_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robmobile_projet_generate_messages_py geometry_msgs_generate_messages_py)
endif()
