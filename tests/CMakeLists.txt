find_package(GTest REQUIRED)
include(GoogleTest)

# TEMPORARY: build smoke test, remove once real tests exist.
add_executable(test_skeleton test_skeleton.cpp)
target_link_libraries(test_skeleton PRIVATE ppmt_fiducial_core GTest::gtest_main)
gtest_discover_tests(test_skeleton)

if(TARGET ppmt_fiducial_synth)
  add_executable(test_synth test_synth.cpp)
  target_link_libraries(test_synth PRIVATE ppmt_fiducial_synth GTest::gtest_main)
  gtest_discover_tests(test_synth)
endif()

# Reads the generated baseline files; `make oracle` produces them.
add_executable(test_oracle test_oracle.cpp)
target_link_libraries(test_oracle PRIVATE ${OpenCV_LIBS} GTest::gtest_main)
target_compile_definitions(test_oracle PRIVATE PPMT_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
gtest_discover_tests(test_oracle)

add_executable(test_configs test_configs.cpp)
target_link_libraries(test_configs PRIVATE ppmt_fiducial_core GTest::gtest_main)
target_compile_options(test_configs PRIVATE -Wall -Wextra -Wpedantic -Werror)
gtest_discover_tests(test_configs)

add_executable(test_fiducial_pipeline test_fiducial_pipeline.cpp)
target_link_libraries(test_fiducial_pipeline PRIVATE ppmt_fiducial_core GTest::gtest_main)
target_compile_options(test_fiducial_pipeline PRIVATE -Wall -Wextra -Wpedantic -Werror)
gtest_discover_tests(test_fiducial_pipeline)

# Compares our 36h11 lookup with OpenCV aruco (the test links aruco; the library does not).
find_package(OpenCV REQUIRED COMPONENTS core aruco)
add_executable(test_april_tag_36h11_dictionary test_april_tag_36h11_dictionary.cpp)
target_link_libraries(test_april_tag_36h11_dictionary PRIVATE
  ppmt_fiducial_core opencv_aruco GTest::gtest_main)
target_compile_options(test_april_tag_36h11_dictionary PRIVATE -Wall -Wextra -Wpedantic -Werror)
gtest_discover_tests(test_april_tag_36h11_dictionary)

# ArucoNano stages: each compared directly with the original aruco_nano on the same input.
if(TARGET ppmt_fiducial_oracle_access)
  foreach(stage threshold contour_tracer quad_candidates)
    add_executable(test_aruco_nano_${stage} aruco_nano/test_${stage}.cpp)
    target_link_libraries(test_aruco_nano_${stage} PRIVATE
      ppmt_fiducial_core ppmt_fiducial_oracle_access GTest::gtest_main)
    target_compile_definitions(test_aruco_nano_${stage} PRIVATE
      PPMT_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
    target_compile_options(test_aruco_nano_${stage} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    gtest_discover_tests(test_aruco_nano_${stage})
  endforeach()
endif()
