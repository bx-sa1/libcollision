PREFIX = /usr/local

.PHONY install:
install: libcollision.hpp
	cp $< $(PREFIX)/include

.PHONY uninstall:
uninstall:
	rm -f $($PREFIX)/include/libcollision.hpp