FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += "file://densitron-weston.ini"

do_install:append () {
    install -m 0644 ${WORKDIR}/densitron-weston.ini ${D}${sysconfdir}/xdg/weston/weston.ini
}
