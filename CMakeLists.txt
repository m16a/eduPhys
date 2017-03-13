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

  set(application_SRCS  main.cpp gpuhelper.cpp icosphere.cpp camera.cpp application.cpp box.cpp phys_ent.cpp my_utils.cpp  sphere.cpp core.cpp obj_mover.cpp debug_draw.cpp poly34.cpp collision.cpp) 

	add_subdirectory("${PROJECT_SOURCE_DIR}/serialization")

  qt4_automoc(${application_SRCS})

  add_executable(eduPhys ${application_SRCS})

# for anti stack-smash
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-stack-protector")
#	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-stack-protector")
#	SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fno-stack-protector")

# for gprof profiling
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
#	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#	SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

# set(CMAKE_BUILD_TYPE Debug)

  target_link_libraries(eduPhys
    ${QT_QTCORE_LIBRARY}    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}  ${OPENGL_LIBRARIES} ${PROTOBUF_LIBRARIES} )

else()

  message(STATUS "OpenGL demo disabled because Qt4 and/or OpenGL have not been found.")

endif()
