SUMMARY = "RTC sync systemd services"
DESCRIPTION = "Restore RTC at boot and sync after NTP"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

inherit systemd

SRC_URI = " \
    file://hwclock-restore.service \
    file://hwclock-sync.service \
    file://hwclock-sync.path \
"

SYSTEMD_SERVICE:${PN} = "hwclock-restore.service hwclock-sync.path"
SYSTEMD_AUTO_ENABLE = "enable"

RDEPENDS:${PN} = "util-linux-hwclock"

do_install() {
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/hwclock-restore.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/hwclock-sync.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/hwclock-sync.path ${D}${systemd_system_unitdir}/
}

FILES:${PN} = "${systemd_system_unitdir}/*"
