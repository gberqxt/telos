FILESEXTRAPATHS:prepend := "${THISDIR}/files:"


# AAEON patches
SRC_URI += "file://0001-AAEON-patch-3.2-to-3.7.patch"


# Add audio codec driver files
SRC_URI += "file://sound/soc/codecs/inf1000.c "

# Add the patch for Kconfig and Makefile
SRC_URI += "file://0001-Add-inf1000-to-Kconfig-and-Makefile.patch "

# Add patch for SAI3 root clock
SRC_URI += "file://0001-Added-clock-for-sai3.patch "

# Include board specific DTS file
SRC_URI += "file://imx8mp-densitron.dts "

# Add SDMA firmware config fragment and PM patch

## # Try to patch SAI3 driver issue
# FIX: Solved loading iMx7 driver from DTS
# SRC_URI += "file://0001-imx-sdma-fix-atomic-context-pm-runtime.patch " 



# Add kernel configuration fragments
SRC_URI += " \
    file://imx8mp-densitron.cfg \
    file://ti-tlv320aic3x.cfg \
    file://inf1000.cfg \
    file://sdma-firmware.cfg \
"

# Ensure firmware is available during kernel build
DEPENDS += "firmware-imx"

# Function to copy the driver files to the kernel source
do_configure:prepend() {
    # Create directory if it doesn't exist
    mkdir -p ${S}/sound/soc/codecs/
    
    # Copy the driver files
    cp ${WORKDIR}/sound/soc/codecs/inf1000.c ${S}/sound/soc/codecs/
    
    # Ensure all kernel config fragments are applied
    cat ${WORKDIR}/inf1000.cfg >> ${B}/.config
    cat ${WORKDIR}/sdma-firmware.cfg >> ${B}/.config
    cat ${WORKDIR}/imx8mp-densitron.cfg >> ${B}/.config
    cat ${WORKDIR}/ti-tlv320aic3x.cfg >> ${B}/.config

    # Replace EVK DTS with our custom DTS - dirty but fast. 
    #     Fixme: create specific 'densitron' target machine
    cp ${WORKDIR}/imx8mp-densitron.dts ${S}/arch/arm64/boot/dts/freescale/imx8mp-evk.dts
    
    # Copy SDMA firmware to kernel source for embedding
    # Ensure the kernel config fragment is applied
    cat ${WORKDIR}/sdma-firmware.cfg >> ${B}/.config

    mkdir -p ${S}/firmware/imx/sdma
    if [ -f ${STAGING_DIR_HOST}${nonarch_base_libdir}/firmware/imx/sdma/sdma-imx7d.bin ]; then
        cp ${STAGING_DIR_HOST}${nonarch_base_libdir}/firmware/imx/sdma/sdma-imx7d.bin ${S}/firmware/imx/sdma/
    fi
}
