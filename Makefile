CONTIKI_PROJECT = group-project

ifdef COOJA
TARGET ?= sky
CFLAGS += -DCOOJA
else
TARGET ?= dpp-cc430
endif

CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)

# select the data rate
CFLAGS += -DDATARATE=10
# select the sink address
CFLAGS += -DSINK_ADDRESS=22
# select the sink address
CFLAGS += -DRANDOM_SEED=123

PROJECT_SOURCEFILES += basic-radio.c data-generator.c

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

ifeq ($(TARGET),dpp-cc430)
login: $(CONTIKI_PROJECT).login
endif

reset: $(CONTIKI_PROJECT).reset

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
CONTIKI = ../..
include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
