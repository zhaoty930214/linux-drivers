ifndef EXAMPLES_MAKEFILE_
EXAMPLES_MAKEFILE_=included

EXAMPLE_DIR = example
EXAMPLE_FILE = axis_get.c axis_getc.c app.c

EXAMPLE_TARGETS = $(EXAMPLE_FILE:%.c=%)
EXAMPLE_CLEAN_TARGETS = $(addsuffix _clean, $(EXAMPLE_TARGETS))
EXAMPLE_EXECUTABLES = $(addprefix $(EXAMPLE_DIR)/,$(EXAMPLE_TARGETS))
EXAMPLE_OUTPUT_EXECUTABLES = $(addprefix $(OUTPUT_DIR)/,$(EXAMPLE_TARGETS))

GLOBAL_CFLAGS = -Wall -Wextra  -Wno-int-to-pointer-cast -Wno-int-conversion -std=gnu99 -g -O0

.PHONY: all $(EXAMPLE_TARGETS)

.SECONDEXPANSION:

axis: $(EXAMPLE_TARGETS)

$(EXAMPLE_TARGETS): $(OUTPUT_DIR)/$$@
	@echo $(OUTPUT_DIR)/$@

$(EXAMPLE_EXECUTABLES): $$@.c
	echo $@.c
	$(CC) $(GLOBAL_CFLAGS) $(filter %.c,$^) -o $@


#$(EXAMPLE_EXECUTABLES): $(EXAMPLE_DIR)/$$(shell basename $@)
#	@cp $< $@

# Copy a compiled example executable to the specified output directory
$(EXAMPLE_OUTPUT_EXECUTABLES): $(EXAMPLE_DIR)/$$(shell basename $$@) \
							   $(OUTPUT_DIR)
	@cp $< $@
	@echo $@

# .phone: $(EXAMPLE_EXECUTABLES)

all:
	@echo $(EXAMPLE_TARGETS)
	@echo $(EXAMPLE_CLEAN_TARGETS)
	@echo $(EXAMPLE_EXECUTABLES)
	@echo $(EXAMPLE_OUTPUT_EXECUTABLES)

endif