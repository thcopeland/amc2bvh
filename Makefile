CC=gcc
CFLAGS=-Wall -Wextra -Wno-implicit-fallthrough -Wno-unused-parameter -flto -O2 -I. -lm -lattyr
DEPS=amc2bvh.h hashmap.h render_test.c
OBJ=amc2bvh.o hashmap.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

amc2bvh: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm $(OBJ) amc2bvh
