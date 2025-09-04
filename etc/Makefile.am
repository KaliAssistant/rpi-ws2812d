# etc/Makefile.am

# Install into $(sysconfdir)/ws2812d/
# sysconfdir = $(prefix)/etc
ws2812ddir = $(sysconfdir)/ws2812d

# Files to install
dist_ws2812d_DATA = ws2812d.conf.sample

# After installing, copy it to ws2812d.conf if it doesn’t exist
install-data-hook:
	@if test ! -f "$(DESTDIR)$(ws2812ddir)/ws2812d.conf"; then \
	  echo "Installing default ws2812d.conf"; \
	  $(MKDIR_P) "$(DESTDIR)$(ws2812ddir)"; \
	  cp "$(DESTDIR)$(ws2812ddir)/ws2812d.conf.sample" \
	     "$(DESTDIR)$(ws2812ddir)/ws2812d.conf"; \
	fi
