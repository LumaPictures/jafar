# $Id$ #

MODULESDIR = $(filter-out $(JAFAR_DIR)/modules $(JAFAR_DIR)/modules/.svn ,$(shell find $(JAFAR_DIR)/modules -maxdepth 1 -type d -print))

all: $(MODULESDIR) doc

clean: $(MODULESDIR)

test: $(MODULESDIR)

.PHONY: $(MODULESDIR) doc
$(MODULESDIR):
	(cd $@; $(MAKE) $(MAKECMDGOALS))

doc:
	doxygen
	for m in $(MODULESDIR); do \
	cp $$m/doc/images/* $(JAFAR_DIR)/doc/html/; \
	done

