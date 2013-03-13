# Common
obj-$(CONFIG_FSL_DPA)				+= dpaa_resource.o

obj-$(CONFIG_FSL_BMAN)				+= bman.o
obj-$(CONFIG_FSL_BMAN_PORTAL)			+= bman-portal.o
bman-portal-y					 = bman_portal.o bman_api.o	\
						   bman_utils.o
obj-$(CONFIG_FSL_BMAN_TEST)			+= bman-test.o
bman-test-y					 = bman_test.o
bman-test-$(CONFIG_FSL_BMAN_TEST_API)		+= bman_test_api.o
bman-test-$(CONFIG_FSL_BMAN_TEST_THRESH)	+= bman_test_thresh.o
obj-$(CONFIG_FSL_BMAN_DEBUGFS)			+= bman-debugfs.o

obj-$(CONFIG_FSL_QMAN)				+= qman_api.o qman_utils.o qman_driver.o
obj-$(CONFIG_FSL_QMAN_CONFIG)			+= qman.o qman_portal.o
obj-$(CONFIG_FSL_QMAN_TEST)			+= qman-test.o
qman-test-y					 = qman_test.o
qman-test-$(CONFIG_FSL_QMAN_TEST_API)		+= qman_test_api.o
qman-test-$(CONFIG_FSL_QMAN_TEST_STASH)		+= qman_test_stash.o
