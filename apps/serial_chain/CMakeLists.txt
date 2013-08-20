###############################################
# apps/viewer

project(serial_chain)
file(GLOB serial_chain_srcs "*.cpp")
file(GLOB serial_chain_hdrs "*.h")
add_executable(serial_chain ${serial_chain_srcs} ${serial_chain_hdrs})
target_link_libraries(serial_chain dart ${DARTExt_LIBRARIES})
set_target_properties(serial_chain PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
