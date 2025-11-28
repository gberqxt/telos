SETUPTOOLS_BUILD_ARGS:append = " --iptables-dir /usr/sbin"
SETUPTOOLS_INSTALL_ARGS:append = " --iptables-dir /usr/sbin"

# Unset the old variables to prevent conflicts
DISTUTILS_BUILD_ARGS:remove = " --iptables-dir /usr/sbin"
DISTUTILS_INSTALL_ARGS:remove = " --iptables-dir /usr/sbin"
