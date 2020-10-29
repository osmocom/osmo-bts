DOCBOOKS = $(foreach v,$(VARIANTS),vty/osmobts-$(v)-vty-reference.xml)
DOCBOOKS_DEPS = $(DOCBOOKS) $(addsuffix .inc,$(DOCBOOKS))
INC_DIR = $(abspath $(builddir)/vty)

include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.docbook.inc

CLEAN_FILES += $(DOCBOOKS_DEPS)
CLEAN_FILES += $(addsuffix .inc.gen,$(DOCBOOKS))
CLEAN_FILES += $(addsuffix .inc.merged,$(DOCBOOKS))

$(INC_DIR):
	mkdir -p $@

vty/osmobts-%-vty-reference.xml: $(top_srcdir)/src/osmo-bts-% $(INC_DIR)
	sed -e "s|@@GENERATED@@|$@.inc|" \
	    -e "s|@@VARIANT@@|$(notdir $<)|" \
	    -e "s|@@REV_NUMBER@@|$(VERSION)|" \
	    -e "s|@@REV_DATE@@|$(shell date +"%dth %B %Y")|" \
	    -e "s|@@CR_YEAR@@|$(shell date +"%Y")|" \
		$(srcdir)/vty/osmobts-vty-reference.xml > $@

vty/osmobts-%-vty-reference.xml.inc: $(top_builddir)/src/osmo-bts-*/osmo-bts-% \
				     $(OSMO_GSM_MANUALS_DIR)/common/vty_additions.xml \
				     $(OSMO_GSM_MANUALS_DIR)/common/chapters/vty.xml \
				     $(OSMO_GSM_MANUALS_DIR)/vty_reference.xsl \
				     $(srcdir)/vty/*.xml $(INC_DIR)
	# a) Invoke osmo-bts-% to generate the list of commands first
	$< --vty-ref-mode default --vty-ref-xml > "$@.gen"
	# ... filter garbage potentially printed by libraries to stdout
	sed -i '/^<vtydoc/,$$!d' "$@.gen"
	# b) Merge the result of a) with global and local additions
	$(OSMO_GSM_MANUALS_DIR)/build/vty_reference_combine.sh \
		$(realpath $(OSMO_GSM_MANUALS_DIR)/merge_doc.xsl) "$@.gen" \
		$(OSMO_GSM_MANUALS_DIR)/common/vty_additions.xml \
		$(srcdir)/vty/*additions*.xml > "$@.merged"
	# c) Convert the result of b) into a valid docbook
	xsltproc $(OSMO_GSM_MANUALS_DIR)/vty_reference.xsl "$@.merged" > $@
