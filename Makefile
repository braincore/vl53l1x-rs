.PHONY: clean

SRC = st-api
LIB = libvl53l1x_api
DYLIB = $(LIB).so
STATICLIB = $(LIB).a
LDFLAGS :=
CFLAGS := -Ist-api/core -Ist-api/platform -std=c99

# Default to build directory if no OUT_DIR specified.
# This is useful for testing outside of cargo.
OUT_DIR := $(if $(OUT_DIR), $(OUT_DIR), build)

SRCS := $(wildcard $(SRC)/*/*.c)
OBJS_SUFFIX := $(SRCS:.c=.o)
OBJS := $(addprefix $(OUT_DIR)/, $(OBJS_SUFFIX))

$(DYLIB): $(OBJS)
		$(CC) $(LDFLAGS) $(CFLAGS) -fPIC -shared -o $(OUT_DIR)/$@ $(OBJS)

$(STATICLIB): $(OBJS)
		ar rcs $(OUT_DIR)/$@ $(OBJS)

$(OUT_DIR)/$(SRC)/%.o: $(SRC)/%.c
		mkdir -p $(dir $@)
		$(CC) $(LDFLAGS) $(CFLAGS) -fPIC -c -o $@ $<

clean:
	rm -f $(OUT_DIR)/$(DYLIB) $(OUT_DIR)/$(STATICLIB) $(OBJS)
