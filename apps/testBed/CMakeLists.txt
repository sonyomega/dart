###############################################
# apps/testBed
project(testBed)
file(GLOB testBed_srcs "*.cpp")
file(GLOB testBed_hdrs "*.h")
add_executable(testBed ${testBed_srcs} ${testBed_hdrs})
target_link_libraries(testBed dart ${DARTExt_LIBRARIES})
set_target_properties(testBed PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
