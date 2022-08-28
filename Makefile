# HACK: We need a unique ID for the verilog files so they can exist harmoniously with the rest of the
# designs that are submitted. So, even if we're not using Wokwi, copy the template project and enter
# the ID below as the WOKWI_PROJECT_ID so we get a guaranteed unique ID:
WOKWI_PROJECT_ID=341244947682296404

fetch:
	sed -e 's/USER_MODULE_ID/$(WOKWI_PROJECT_ID)/g' template/scan_wrapper.v > src/scan_wrapper_$(WOKWI_PROJECT_ID).v
	echo $(WOKWI_PROJECT_ID) > src/ID

# needs PDK_ROOT and OPENLANE_ROOT, OPENLANE_IMAGE_NAME set from your environment
harden:
	docker run --rm \
	-v $(OPENLANE_ROOT):/openlane \
	-v $(PDK_ROOT):$(PDK_ROOT) \
	-v $(CURDIR):/work \
	-e PDK_ROOT=$(PDK_ROOT) \
	-u $(shell id -u $(USER)):$(shell id -g $(USER)) \
	$(OPENLANE_IMAGE_NAME) \
	/bin/bash -c "./flow.tcl -overwrite -design /work/src -run_path /work/runs -tag wokwi"
