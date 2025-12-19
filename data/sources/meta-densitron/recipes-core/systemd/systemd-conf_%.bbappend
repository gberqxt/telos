FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " \
    file://10-eth0.network \
    file://10-eth1.network \
"

FILES:${PN}:append = " \
    ${sysconfdir}/systemd/network/10-eth0.network \
    ${sysconfdir}/systemd/network/10-eth1.network \
"

do_install:append() {
    install -d ${D}${sysconfdir}/systemd/network
    install -m 0644 ${WORKDIR}/eth0.network ${D}${sysconfdir}/systemd/network/
    install -m 0644 ${WORKDIR}/eth1.network ${D}${sysconfdir}/systemd/network/
}

