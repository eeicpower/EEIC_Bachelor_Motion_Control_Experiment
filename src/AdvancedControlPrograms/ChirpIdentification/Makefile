all:  inert_chirp 

clean:
	-rm -f ./inert_chirp

inert_chirp: inert_chirp.o chirp.o
	gcc -Wall -I. -O2 -o inert_chirp inert_chirp.o chirp.o /usr/lib/art_syscalls.o -lgpg3100 -lgpg3300

inert_chirp.o: inert_chirp.c
	gcc -c inert_chirp.c 

chirp.o: chirp.c
	gcc -c chirp.c
