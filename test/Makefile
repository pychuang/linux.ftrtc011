CFLAGS= -static
CC=arm-none-linux-gnueabi-gcc

rtctst: rtctst.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f rtctst
