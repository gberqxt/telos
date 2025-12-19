SUMMARY = "Ethernet MAC and LED configuration"
LICENSE = "CLOSED"

SRC_URI = " \
    file://lancfg.sh \
    file://ethernet.service \
    file://mdio-tool \
    file://srgimx8cfg \
"

S = "${WORKDIR}"

inherit systemd

SYSTEMD_SERVICE:${PN} = "ethernet.service"
SYSTEMD_AUTO_ENABLE = "enable"

do_install() {
    # Install scripts
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/lancfg.sh ${D}${bindir}/
    install -m 0755 ${WORKDIR}/mdio-tool ${D}${bindir}/
    install -m 0755 ${WORKDIR}/srgimx8cfg ${D}${bindir}/
    
    # Install systemd service
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/ethernet.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} += "${bindir}/* ${systemd_system_unitdir}/*"

RDEPENDS:${PN} = "bash i2c-tools"
