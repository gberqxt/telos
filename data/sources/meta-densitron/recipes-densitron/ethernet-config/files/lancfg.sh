#!/bin/bash

function write_LAN_mac_address()
{
    error_mac1="00:00:00:00:00:00"
    error_mac2="FF:FF:FF:FF:FF:FF"

    O_ETH0=$(ifconfig -a eth0 | awk '/ether/ { print $2 }')
    O_ETH1=$(ifconfig -a eth1 | awk '/ether/ { print $2 }')

    ETH0_TMP=$(/usr/bin/srgimx8cfg -0)
    ETH1_TMP=$(/usr/bin/srgimx8cfg -1)
    
    calcheckSum=$(/usr/bin/srgimx8cfg -p)
    readcheckSum=$(/usr/bin/srgimx8cfg -k)

    if [[ "$ETH0_TMP" == "$error_mac1" ]] || [[ "$ETH0_TMP" == "$error_mac2" ]]; then
        sleep 1
        return
    elif [[ "$ETH1_TMP" == "$error_mac1" ]] || [[ "$ETH1_TMP" == "$error_mac2" ]]; then
        sleep 1
        return
    elif [ "$calcheckSum" != "$readcheckSum" ]; then
        sleep 1
        return
    else
        sleep 0.3
        ifconfig eth0 down
        ifconfig eth1 down
        
        ip link set dev eth0 address ${ETH0_TMP}
        ip link set dev eth1 address ${ETH1_TMP}
        
        sleep 1.2
        ifconfig eth0 up
        ifconfig eth1 up
    fi
}

function write_LAN_LED_setting()
{
    /usr/bin/mdio-tool w eth0 0x1f 0x0d04
    /usr/bin/mdio-tool w eth0 0x10 0x6260
    /usr/bin/mdio-tool w eth0 0x1f 0x0000
    /usr/bin/mdio-tool w eth1 0x1f 0x0d04
    /usr/bin/mdio-tool w eth1 0x10 0x6260
    /usr/bin/mdio-tool w eth1 0x1f 0x0a42
}

function main()
{
    write_LAN_mac_address
    sleep 2
    write_LAN_LED_setting
    i2ctransfer -f -y 2 w2@0x51 0x02 0x08
}

main
