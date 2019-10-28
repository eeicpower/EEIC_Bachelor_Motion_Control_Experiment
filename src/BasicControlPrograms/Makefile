all: pd4 dist4 inert_internal inert_external

clean:
	-rm -f ./pd4
	-rm -f ./dist4
	-rm -f ./inert_internal
	-rm -f ./inert_external

pd4: pd4.c
	gcc -Wall -O2 -o pd4 pd4.c /usr/lib/art_syscalls.o -lgpg3100 -lgpg3300

dist4: dist4.c
	gcc -Wall -O2 -o dist4 dist4.c /usr/lib/art_syscalls.o -lgpg3100 -lgpg3300

inert_internal: inert_internal.c
	gcc -Wall -O2 -o inert_internal inert_internal.c /usr/lib/art_syscalls.o -lgpg3100 -lgpg3300

inert_external: inert_external.c
	gcc -lrt -Wall -O2 -o inert_external inert_external.c /usr/lib/art_syscalls.o -lgpg3100 -lgpg3300
