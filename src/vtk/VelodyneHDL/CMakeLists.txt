
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


find_library(PCAP_LIBRARY pcap DOC "pcap library")
find_path(PCAP_INCLUDE_DIR pcap.h DOC "pcap include directory")
mark_as_advanced(PCAP_LIBRARY PCAP_INCLUDE_DIR)
include_directories(${PCAP_INCLUDE_DIR})



set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread regex date_time)
include_directories(${Boost_INCLUDE_DIRS})



set(sources
  vtkVelodyneHDLReader.cxx
  vtkVelodyneHDLSource.cxx
  )

set(VTK_LIBRARIES
  vtkFiltering
  vtkIO
  )

set(deps
  ${PCAP_LIBRARY}
  ${Boost_LIBRARIES}
  ${VTK_LIBRARIES}
  )


set(library_name vtkVelodyneHDL)


add_library(${library_name} ${sources})
target_link_libraries(${library_name} ${deps})

add_executable(PacketFileSender PacketFileSender.cxx)
target_link_libraries(PacketFileSender ${deps})

add_executable(TestReader test/TestReader.cxx)
target_link_libraries(TestReader ${library_name})




install(TARGETS ${library_name}
    RUNTIME DESTINATION ${DD_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${DD_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${DD_INSTALL_LIB_DIR})

if(VTK_WRAP_PYTHON)
  include_directories(${VTK_PYTHON_INCLUDE_DIR})
  include(${CMAKE_SOURCE_DIR}/cmake/wrap-python.cmake)
  wrap_python(${library_name} "${sources}")
  set_target_properties(${library_name}Python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DD_INSTALL_PYTHON_DIR}/ddapp")
endif()
