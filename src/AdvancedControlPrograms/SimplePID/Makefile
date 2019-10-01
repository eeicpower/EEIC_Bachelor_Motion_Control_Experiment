all: pid

clean:
	-rm -f ./pid

pid: pid.c
	gcc -Wall -O2 -o pid pid.c /usr/lib/art_syscalls.o -lgpg3100 -lgpg3300