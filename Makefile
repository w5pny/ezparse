BUILD_DIR	:= build
SRC		:= ezparse.c
OBJ		:= $(SRC:%.c=$(BUILD_DIR)/%.o)
CFLAGS		:= -g -O2
LDFLAGS		:= -lm

all: $(BUILD_DIR) $(BUILD_DIR)/ezparse

$(BUILD_DIR)/ezparse: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

clean:
	rm -fr build
