CC=gcc
CFLAGS=-Wall -Wextra -Wno-implicit-fallthrough -Wno-unused-parameter -flto -O2 -I. -lm
DEPS=amc2bvh.h matrices.h hashmap.h
OBJ=amc2bvh.o matrices.o hashmap.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

amc2bvh: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm $(OBJ) amc2bvh
