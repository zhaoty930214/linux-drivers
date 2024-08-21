
SHELL = /bin/bash

CC=$(CROSS_COMPILE)gcc -g

OUTPUT_DIR?=output

include example/example.mk


.PHONY: all $(OUTPUT_DIR)

$(OUTPUT_DIR):
	@mkdir -p $(OUTPUT_DIR)