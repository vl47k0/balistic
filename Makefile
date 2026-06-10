# balistic — portable tennis ballistics core.
# Pure C11, math.h only. No GTK / glib / json-glib. Builds a static lib +
# headers; `make test` runs the math-core assertions with no GUI toolkit.

CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra -std=c11
# -Iinclude        → consumers' <ballistic/physics.h>
# -Iinclude/ballistic → the library's own flat #include "physics.h"
INCLUDE := -Iinclude -Iinclude/ballistic

SRCS    := src/physics.c src/strike.c src/toss.c src/pose.c
OBJS    := $(SRCS:.c=.o)
HEADERS := include/ballistic/physics.h include/ballistic/strike.h \
           include/ballistic/toss.h include/ballistic/pose.h \
           include/ballistic/ballistic.h

LIB     := build/libballistic.a
TESTBIN := build/balistic_test

# Install prefix (for `make install`): headers → $(PREFIX)/include/ballistic,
# lib → $(PREFIX)/lib. Consumers in this umbrella usually skip install and just
# point at this tree: -Iprojects/balistic/include -Lprojects/balistic/build.
PREFIX  ?= /usr/local

all: $(LIB)

$(LIB): $(OBJS)
	@mkdir -p build
	$(AR) rcs $@ $(OBJS)

src/%.o: src/%.c $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

# `make debug` — -O0 + asserts so backtraces line up with source.
debug:
	$(MAKE) clean
	$(MAKE) CFLAGS="-O0 -g3 -fno-omit-frame-pointer -Wall -Wextra -std=c11"

# The math core links against nothing but libm.
$(TESTBIN): tests/test.c $(LIB)
	@mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDE) tests/test.c $(LIB) -o $@ -lm

test: $(TESTBIN)
	./$(TESTBIN)

install: $(LIB)
	install -d $(PREFIX)/include/ballistic $(PREFIX)/lib
	install -m644 $(HEADERS) $(PREFIX)/include/ballistic/
	install -m644 $(LIB) $(PREFIX)/lib/

clean:
	rm -f $(OBJS) $(LIB) $(TESTBIN)

.PHONY: all debug test install clean
