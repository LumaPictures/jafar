# $Id$ #

MODULESDIR := $(filter-out $(JAFAR_DIR)/modules $(JAFAR_DIR)/modules/.svn ,$(shell find $(JAFAR_DIR)/modules -maxdepth 1 -type d -print))

.PHONY: doc

# template

define TARGET_template

$(1)_MODULES := $$(subst $(JAFAR_DIR)/modules/,$(1)_,$(MODULESDIR))
$(1)M_MODULES := $$(subst $(JAFAR_DIR)/modules/,$(1)m_,$(MODULESDIR))
.PHONY: $(1) $$($(1)_MODULES) $$($(1)M_MODULES)
$(1):
	for m in $$(shell ${JAFAR_DIR}/bin/dep.sh $$(MODULESDIR)); do \
	$$(MAKE) $(1)m_$$$$m || exit; \
	done
#	+$(MAKE) $$(addprefix $(1)m_,$$(shell ${JAFAR_DIR}/bin/dep.sh $$(MODULESDIR)))	

$$($(1)_MODULES): 
	for m in $$(shell ${JAFAR_DIR}/bin/dep.sh $$(subst $(1)_,,$$@)); do \
	$$(MAKE) $(1)m_$$$$m || exit; \
	done
#	+$(MAKE) $$(addprefix $(1)m_,$$(shell ${JAFAR_DIR}/bin/dep.sh $$(subst $(1)_,,$$@)))

#$$($(1)M_MODULES): $$(shell echo $(MAKECMDGOALS) | sed 's/$$@.*//g')
$$($(1)M_MODULES):
	+(cd $(JAFAR_DIR)/modules/$$(subst $(1)m_,,$$@); $(MAKE) $(1)-mod-a)

endef

# all, lib, clean, test, ruby

$(eval $(call TARGET_template,all))
$(eval $(call TARGET_template,lib))
$(eval $(call TARGET_template,clean))
$(eval $(call TARGET_template,test))
$(eval $(call TARGET_template,script))

# doc
doc:
	doxygen
	for m in $(MODULESDIR); do \
	cp $$m/doc/images/* $(JAFAR_DIR)/doc/html/; \
	done

all-old: $(MODULESDIR)
	@echo $(MODULESDIR)

.PHONY: $(MODULESDIR)
$(MODULESDIR):
	(cd $@; $(MAKE) $(MAKECMDGOALS))



