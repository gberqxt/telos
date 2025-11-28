# Override the source URL
NNSHARK_SRC = "git://github.com/nxp-imx/nnshark.git;protocol=https"

# Add required utilities for git operations
DEPENDS += "git-native"

do_unpack[postfuncs] += "setup_git_submodule"

setup_git_submodule() {
    cd ${S}
    if [ -d .git ]; then
        bbnote "Configuring git submodule URL..."
        git config submodule.common.url https://github.com/GStreamer/common.git
        
        bbnote "Updating git submodules..."
        git submodule update --init --recursive
        
        if [ -f common/gst-autogen.sh ]; then
            bbnote "Successfully verified common/gst-autogen.sh exists"
        else
            bbfatal "Failed to find common/gst-autogen.sh after submodule update"
        fi
    else
        bbfatal "Not a git repository: ${S}"
    fi
}

do_configure:prepend() {
    if [ ! -f ${S}/common/gst-autogen.sh ]; then
        bbfatal "Required files not found in common directory"
    fi
}
