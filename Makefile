OBJS=main.o

TARGET=bladeout

CC=gcc

CFLAGS=-O3 -pipe -std=gnu99 -march=native -mtune=native -frename-registers -funroll-loops -ftree-vectorize -Wall -pedantic
CFLAGS+=$(shell pkg-config --cflags libbladeRF)

LDFLAGS=-lpthread -lm -lbladeRF
LDFLAGS+=$(shell pkg-config --libs libbladeRF)

all: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
