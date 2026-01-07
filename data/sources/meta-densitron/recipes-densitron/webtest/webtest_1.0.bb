SUMMARY = "Web-based Test Interface"
DESCRIPTION = "Flask web interface for audio, display and touch panel testing"
LICENSE = "CLOSED"

SRC_URI = "file://app_backend.py \
           file://app.py \
           file://app_ui.ui \
           file://connector_test.sh \
           file://coordinates.csv \
           file://README.md \
           file://requirements.txt \
           file://wavip.py \
           file://templates/display_monitor.html \
           file://templates/index.html \
           file://sound/chirp_sine_100-10kHz_16bit_48000-stereo.wav \
           file://sound/chirp_sine_100-10kHz_24bit_48000-stereo.wav \
           file://sound/chirp_sine_100-10kHz_24bit_96000-stereo.wav \
           file://sound/chirp_sine_100-10kHz_32bit_32000-stereo.wav \
           file://sound/1000Hz_stereo_16bit-signed_48kHz.wav \
           file://sound/Fytch-Fading_light.wav \
           file://webtest.service \
          "

# Skip checksum verification for local files
BB_STRICT_CHECKSUM = "0"

S = "${WORKDIR}"

RDEPENDS:${PN} = "python3 python3-flask python3-evdev python3-pygobject gtk+3"

inherit systemd

SYSTEMD_SERVICE:${PN} = "webtest.service"
SYSTEMD_AUTO_ENABLE = "enable"

do_install() {
    # Install application to /home/root/webtest
    install -d ${D}/home/root/webtest
    install -m 0755 ${WORKDIR}/app.py ${D}/home/root/webtest/
    install -m 0644 ${WORKDIR}/app_backend.py ${D}/home/root/webtest/
    install -m 0644 ${WORKDIR}/app_ui.ui ${D}/home/root/webtest/
    install -m 0755 ${WORKDIR}/connector_test.sh ${D}/home/root/webtest/
    install -m 0644 ${WORKDIR}/coordinates.csv ${D}/home/root/webtest/
    install -m 0644 ${WORKDIR}/README.md ${D}/home/root/webtest/
    install -m 0644 ${WORKDIR}/requirements.txt ${D}/home/root/webtest/
    install -m 0755 ${WORKDIR}/wavip.py ${D}/home/root/webtest/


    # Create flag files with correct permissions
    touch ${D}/home/root/webtest/.initialized
    touch ${D}/home/root/webtest/.autostart
    touch ${D}/home/root/webtest/.wavip_autoplay

    # Install templates
    install -d ${D}/home/root/webtest/templates
    install -m 0644 ${WORKDIR}/templates/*.html ${D}/home/root/webtest/templates/

    # Install sound files
    install -d ${D}/home/root/webtest/sound
    install -m 0644 ${WORKDIR}/sound/*.wav ${D}/home/root/webtest/sound/

    # Install systemd service
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/webtest.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} += "/home/root/webtest/* \
                /home/root/webtest/.autostart \
                /home/root/webtest/.wavip_autoplay \
                /home/root/webtest/.initialized \
               "
