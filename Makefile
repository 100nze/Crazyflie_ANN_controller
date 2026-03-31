CRAZYFLIE_BASE := $(PWD)/external/crazyflie-firmware

OOT_CONFIG := $(PWD)/config

#where other .h files are
EXTRA_CFLAGS += -I$(PWD)/src/ST_nn/
EXTRA_CFLAGS += -I/opt/ST/STEdgeAI/4.0/Middlewares/ST/AI/Inc
EXTRA_CFLAGS += -I$(PWD)/src

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
