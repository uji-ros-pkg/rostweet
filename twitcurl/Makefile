all: installed

SVN_DIR = build/twitcurl
SVN_URL = http://twitcurl.googlecode.com/svn/trunk/
SVN_REVISION = -r 74
SVN_PATCH = libtwitcurl.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR)/libtwitcurl && cmake . && make
	cp $(SVN_DIR)/libtwitcurl/libtwit.a lib/ 
	cp $(SVN_DIR)/libtwitcurl/twitcurl.h $(SVN_DIR)/libtwitcurl/oauthlib.h include/
	touch installed

clean:
	cd $(SVN_DIR)/libtwitcurl && make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)
