BUILD_DIR     := build
SYNTH_OUT_DIR := data/sample/synthetic
ORACLE_OUT_DIR := data/oracle/synthetic

.PHONY: build test synthetic_markers oracle clean

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

clean:
	rm -rf $(BUILD_DIR)
