XGMAC_VERSION = 1.0
XGMAC_SITE = $(BR2_EXTERNAL_XGMAC_PATH)/../../xg_mac_driver
XGMAC_SITE_METHOD = local

$(eval $(kernel-module))
$(eval $(generic-package))