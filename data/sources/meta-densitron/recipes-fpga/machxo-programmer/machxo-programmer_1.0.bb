SUMMARY = "MachXO FPGA Programmer for ARM64"
DESCRIPTION = "Precompiled binary for updating FPGA firmware"
LICENSE = "CLOSED"

SRC_URI = "file://machxo_programmer_arm64"

S = "${WORKDIR}"

# Skip QA checks for pre-built binaries
INSANE_SKIP:${PN} = "already-stripped ldflags"

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/machxo_programmer_arm64 ${D}${bindir}/machxo-programmer
}

FILES:${PN} = "${bindir}/machxo-programmer"
