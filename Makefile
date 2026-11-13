BUILD_DIR     := build
SYNTH_OUT_DIR := data/sample/synthetic
ORACLE_OUT_DIR := data/oracle/synthetic
DEBUG_OUT_DIR := data/debug/synthetic
APRIL_TAG_36H11_CODES := include/ppmt_fiducial_core/algorithms/aruco_nano/april_tag_36h11_codes.hpp

.PHONY: build test synthetic_markers oracle debug_images april_tag_36h11_codes clean

build:
	cmake -S . -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR) -j

test: oracle
	ctest --test-dir $(BUILD_DIR) --output-on-failure

synthetic_markers: build
	mkdir -p $(SYNTH_OUT_DIR)
	$(BUILD_DIR)/tools/ppmt_fiducial_synth_generate $(SYNTH_OUT_DIR)

oracle: synthetic_markers
	mkdir -p $(ORACLE_OUT_DIR)
	$(BUILD_DIR)/tools/ppmt_fiducial_oracle $(SYNTH_OUT_DIR) $(ORACLE_OUT_DIR)

debug_images: synthetic_markers
	$(BUILD_DIR)/tools/ppmt_fiducial_debug_images $(SYNTH_OUT_DIR) $(DEBUG_OUT_DIR)

april_tag_36h11_codes: build
	$(BUILD_DIR)/tools/ppmt_fiducial_export_april_tag_36h11 $(APRIL_TAG_36H11_CODES)

clean:
	rm -rf $(BUILD_DIR)
