ifndef EXAMPLES_MAKEFILE_
EXAMPLES_MAKEFILE_=included

include library/library.mk


EXAMPLE_DIR = example
EXAMPLE_FILE = axis_get.c 
#axis_getc.c app.c

EXAMPLE_CFLAGS = $(GLOBAL_CFLAGS)

EXAMPLE_TARGETS = $(EXAMPLE_FILE:%.c=%)
EXAMPLE_CLEAN_TARGETS = $(addsuffix _clean, $(EXAMPLE_TARGETS))
EXAMPLE_EXECUTABLES = $(addprefix $(EXAMPLE_DIR)/,$(EXAMPLE_TARGETS))
EXAMPLE_OUTPUT_EXECUTABLES = $(addprefix $(OUTPUT_DIR)/,$(EXAMPLE_TARGETS))



# Set the example executables to link against the AXI DMA shared library in
# the outputs directory
#EXAMPLES_LINKER_FLAGS = -Wl,-rpath,'$$ORIGIN'
EXAMPLES_LIB_FLAGS = -L $(OUTPUT_DIR) -l $(LIBMPU6050_NAME) -lm \
					 $(EXAMPLES_LINKER_FLAGS)


.PHONY: $(EXAMPLE_TARGETS) $(EXAMPLE_CLEAN_TARGETS)

.SECONDEXPANSION:

examples: $(EXAMPLE_TARGETS)

$(EXAMPLE_TARGETS): $(OUTPUT_DIR)/$$@
#@echo $(OUTPUT_DIR)/$@

$(EXAMPLE_EXECUTABLES): $$@.c
	echo $@.c
	$(CC) $(EXAMPLE_CFLAGS) $(LIBAXIDMA_INC_FLAGS)  \
	 $(filter %.c,$^) -lm -o $@ $(EXAMPLES_LIB_FLAGS)

examples_clean: $(EXAMPLE_CLEAN_TARGETS)

$(EXAMPLE_CLEAN_TARGETS): 
	rm  -f $(EXAMPLE_EXECUTABLES)   $(EXAMPLE_OUTPUT_EXECUTABLES)


#$(EXAMPLE_EXECUTABLES): $(EXAMPLE_DIR)/$$(shell basename $@)
#	@cp $< $@

# Copy a compiled example executable to the specified output directory
$(EXAMPLE_OUTPUT_EXECUTABLES): $(EXAMPLE_DIR)/$$(shell basename $$@) \
							   $(OUTPUT_DIR)
	@cp $< $@
#@echo $@

# .phone: $(EXAMPLE_EXECUTABLES)


endif