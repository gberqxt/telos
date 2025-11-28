
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "file://uboot-custom.cfg"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "file://0001-patches-1.2-and-.13-becasue-1.4-was-added-as-fragmen.patch"

FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"
SRC_URI += "file://0001-Modified-files-followoing-AAOPEN-patch-1.1.patch"


