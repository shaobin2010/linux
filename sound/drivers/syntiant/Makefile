mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
mkfile_dir := $(dir $(mkfile_path))

obj-y += syn_ndp.o
syn_ndp-objs += \
	syn_ndp_driver.o \
	syntiant_ndp.o \
	syntiant_ndp10x.o \
	syntiant_ndp120.o \
	syntiant_ndp120_mailbox.o \
	syntiant_ndp120_config.o \
	syntiant_ndp120_config_misc.o \
	syntiant_ndp120_ph.o \
	syntiant_portability.o \
	checksum.o \
	syntiant_package.o \
	syntiant_ndp10x_package_parser.o \
	syntiant_ndp_error.o \
	syntiant_ndp_common.o \
	syntiant_ndp_aux.o \
	es_conversion.o

$(info "1111111   "$(mkfile_dir))



ccflags-y:=-DSPI_READ_DELAY=3 -DCHIP_TYPE_10X -DMAX_SPEED=10000000
ccflags-y+=-I$(mkfile_dir)/

# setting LIBRARY_PATH (as in env.sh) causes an error:
# ./include/linux/kernel.h:6:20: fatal error: stdarg.h: No such file or directory
# #include <stdarg.h>
# so undefine it
undefine LIBRARY_PATH

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) clean
