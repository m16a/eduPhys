cmake_minimum_required(VERSION 2.8)

#path to default packages: /usr/share/cmake-x.y/Modules
find_package(Qt4)
find_package(OpenGL)
find_package(Protobuf)


if(QT4_FOUND AND OPENGL_FOUND)

  set(QT_USE_QTOPENGL TRUE)
  include(${QT_USE_FILE})

  set(CMAKE_INCLUDE_CURRENT_DIR ON)
	
  include_directories( ${QT_INCLUDE_DIR} )

  set(quaternion_demo_SRCS  gpuhelper.cpp icosphere.cpp camera.cpp trackball.cpp quaternion_demo.cpp box.cpp geometry.cpp my_utils.cpp  sphere.cpp core.cpp obj_mover.cpp debug_draw.cpp poly34.cpp) 

	add_subdirectory("${PROJECT_SOURCE_DIR}/serialization")

  qt4_automoc(${quaternion_demo_SRCS})

  add_executable(quaternion_demo ${quaternion_demo_SRCS})

  target_link_libraries(quaternion_demo
    ${QT_QTCORE_LIBRARY}    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}  ${OPENGL_LIBRARIES} ${PROTOBUF_LIBRARIES} )

else()

  message(STATUS "OpenGL demo disabled because Qt4 and/or OpenGL have not been found.")

endif()
