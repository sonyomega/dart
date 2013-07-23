###############################################
# apps/viewer_DART

project(viewer_DART)
file(GLOB viewer_DART_srcs "*.cpp")
file(GLOB viewer_DART_hdrs "*.h")
add_executable(viewer_DART ${viewer_DART_srcs} ${viewer_DART_hdrs})
target_link_libraries(viewer_DART dart ${DARTExt_LIBRARIES})
set_target_properties(viewer_DART PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
