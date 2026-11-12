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
