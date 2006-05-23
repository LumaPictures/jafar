# $Id$ #

MODULESDIR = $(filter-out $(JAFAR_DIR)/modules $(JAFAR_DIR)/modules/.svn ,$(shell find $(JAFAR_DIR)/modules -maxdepth 1 -type d -print))

all: $(MODULESDIR) doc

clean: $(MODULESDIR)

.PHONY: $(MODULESDIR) doc
$(MODULESDIR):
	(cd $@; $(MAKE) $(MAKECMDGOALS))

doc:
	doxygen
	cp $(JAFAR_DIR)/doc/doxygen/images/LAASsmall.png $(JAFAR_DIR)/doc/html/LAASsmall.png 
