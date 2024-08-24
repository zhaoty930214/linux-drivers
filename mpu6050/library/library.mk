ifndef LIBAXIDMA_MAKEFILE_
LIBAXIDMA_MAKEFILE_=included

LIBMPU6050_DIR = library
LIBMPU6050_FILES = atk_ms6050.c eMPL/inv_mpu.c eMPL/inv_mpu_dmp_motion_driver.c
LIBMPU6050 = $(addprefix $(LIBMPU6050_DIR)/,$(LIBMPU6050_FILES))

LIBMPU6050_NAME = mpu6050
LIBMPU6050_LIBRARY = $(LIBMPU6050_DIR)/lib$(LIBMPU6050_NAME).so
LIBMPU6050_OUTPUT_LIBRARY = $(OUTPUT_DIR)/lib$(LIBMPU6050_NAME).so

LIBMPU6050_INC_DIRS = include
LIBMPU6050_INC_FILES = atk_ms6050.h inv_mpu.h inv_mpu_dmp_motion_driver.h dmpmap.h dmpKey.h
LIBMPU6050_INC = $(addprefix $(LIBMPU6050_INC_DIRS)/,$(LIBMPU6050_INC_FILES))
LIBAXIDMA_INC_FLAGS = $(addprefix -I ,$(LIBMPU6050_INC_DIRS))

# The flags for compiling the library
LIBAXIDMA_CFLAGS = $(GLOBAL_CFLAGS) -fPIC -shared \
				   -Wno-missing-field-initializers

$(LIBMPU6050_LIBRARY): $(LIBMPU6050) $(LIBMPU6050_INC) | cross_compiler_check
	$(CC) $(LIBAXIDMA_CFLAGS) $(LIBAXIDMA_INC_FLAGS) $(filter %.c,$^) -o $@

library: $(LIBMPU6050_OUTPUT_LIBRARY)


library_clean:
	rm -f	$(LIBMPU6050_OUTPUT_LIBRARY)	$(LIBMPU6050_LIBRARY)


$(LIBMPU6050_OUTPUT_LIBRARY): $(LIBMPU6050_LIBRARY) $(OUTPUT_DIR)
	@cp $< $@

endif