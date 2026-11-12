# Tools are NOT part of the core library and may use extra OpenCV modules (aruco, imgcodecs).
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs aruco)

# Synthetic scene renderer, shared by the generator CLI and the tests.
add_library(ppmt_fiducial_synth STATIC synth/synth.cpp)
target_include_directories(ppmt_fiducial_synth PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/synth)
target_link_libraries(ppmt_fiducial_synth PUBLIC ${OpenCV_LIBS})
target_compile_options(ppmt_fiducial_synth PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(ppmt_fiducial_synth_generate synth/generate_main.cpp)
target_link_libraries(ppmt_fiducial_synth_generate PRIVATE ppmt_fiducial_synth)
target_compile_options(ppmt_fiducial_synth_generate PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Oracle: the original aruco_nano (pinned, unmodified), run on images to produce reference
# output. Its header and the OpenCV < 4.7 compat header are SYSTEM includes for this target only.
add_executable(ppmt_fiducial_oracle oracle/oracle_main.cpp)
target_include_directories(ppmt_fiducial_oracle SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/oracle/compat
  ${CMAKE_CURRENT_SOURCE_DIR}/oracle/third_party)
target_link_libraries(ppmt_fiducial_oracle PRIVATE ${OpenCV_LIBS})
target_compile_options(ppmt_fiducial_oracle PRIVATE -Wall -Wextra -Wpedantic -Werror)
