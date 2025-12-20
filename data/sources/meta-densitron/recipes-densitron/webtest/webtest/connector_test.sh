#!/bin/sh
################################################################################
# Connector Test Script - CN5/CN6 Verification
# 
# This script tests all critical connections between CPU board and Audio board
# through connectors CN5 (audio signals) and CN6 (control signals)
#
# CN5 Pin Assignments:
#   Pin 2: SAI.MCLK     - Master clock for audio interface
#   Pin 4: SAI.RX_BCLK  - Receive bit clock
#   Pin 6: SAI.TX_SYNC  - Transmit sync/frame clock
#   Pin 7: SAI_TX_DATA  - Transmit data line
#   Pin 8: SAI.RX_DATA  - Receive data line
#   Pin 9: RST          - FPGA/CPLD reset signal
#
# CN6 Pin Assignments:
#   Pin 2: CPU.SPI_CS0  - SPI chip select 0 (CPLD)
#   Pin 3: CPU_I2C.SCL  - I2C clock line
#   Pin 4: CPU_SPI.MOSI - SPI master out, slave in
#   Pin 5: CPU_I2C.SDA  - I2C data line
#   Pin 6: CPU_SPI.MISO - SPI master in, slave out
#   Pin 7: ENC_INT      - Encoder/CPLD interrupt signal
#   Pin 8: CPU_SPI.SCLK - SPI clock
#   Pin 9: CPU.SPI_CS1  - SPI chip select 1 (PCM1748 DAC)
#
# Device Connections:
#   SPI.0 (CS0) : CPLD/FPGA - LED control, system registers
#   SPI.1 (CS1) : PCM1748 DAC - Audio DAC configuration
#   I2C         : TAS5733L - Audio amplifier
#   SAI Rx/Tx   : Audio data input/output
#   Reset       : CPLD/FPGA hardware reset
#   Interrupt   : CPLD status and event signaling
#
################################################################################

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test results tracking
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_TOTAL=4

################################################################################
# Helper Functions
################################################################################

# Extract 3rd byte from spidev_test output (register value)
get_rx_value() {
    echo "$1" | grep "RX |" | awk '{print $5}'
}

# Enter test mode: unbind audio driver and bind spidev for direct SPI access
testmode_begin() {
    echo "  -> Unbinding densitron-audio driver..."
    echo densitron-audio > /sys/bus/platform/drivers/densitron-audio/unbind 2>/dev/null || true
    sleep 0.3
    
    echo "  -> Binding spidev to spi4.0 (CPLD)..."
    echo spidev > /sys/bus/spi/devices/spi4.0/driver_override
    echo spi4.0 > /sys/bus/spi/drivers/spidev/bind
    
    echo "  -> Binding spidev to spi4.1 (PCM1748)..."
    echo spidev > /sys/bus/spi/devices/spi4.1/driver_override
    echo spi4.1 > /sys/bus/spi/drivers/spidev/bind
}

# Exit test mode: restore original audio driver
testmode_end() {
    echo "  -> Restoring densitron-audio driver..."
    echo spi4.0 > /sys/bus/spi/drivers/spidev/unbind 2>/dev/null || true
    echo "" > /sys/bus/spi/devices/spi4.0/driver_override
    echo spi4.1 > /sys/bus/spi/drivers/spidev/unbind 2>/dev/null || true
    echo "" > /sys/bus/spi/devices/spi4.1/driver_override
    sleep 0.3
    echo densitron-audio > /sys/bus/platform/drivers/densitron-audio/bind 2>/dev/null || true
}

# Print test header
print_test_header() {
    echo ""
    echo "------------------------------------------------------------------"
    echo "$1"
    echo "------------------------------------------------------------------"
}

# Print test result
print_result() {
    if [ "$1" = "PASS" ]; then
        echo "** PASS - $2"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        echo "!! FAIL - $2"
        TESTS_FAILED=$((TESTS_FAILED + 1))
    fi
}

################################################################################
# TEST 1: I2C Communication (TAS5733L Amplifier)
# Tests: CN6 pins 3 (SCL) and 5 (SDA)
################################################################################
test_i2c() {
    print_test_header "TEST 1: I2C Communication (TAS5733L)"
    
    echo "  Testing I2C bus access to TAS5733L audio amplifier..."
    echo "  Expected Device ID: 0x41"
    
    # Read TAS5733L device ID via sysfs
    DEVICE_ID=$(cat /sys/devices/platform/densitron-audio/tas5733_device_id 2>/dev/null || echo "ERROR")
    
    if [ "$DEVICE_ID" = "0x41" ]; then
        print_result "PASS" "I2C communication OK, Device ID: $DEVICE_ID"
        return 0
    else
        print_result "FAIL" "I2C failed, got: $DEVICE_ID"
        return 1
    fi
}

################################################################################
# TEST 2: FPGA Reset, SPI Communication, and Interrupt
# Tests: CN6 pins 2,4,6,8,9 (SPI.0) and CN5 pin 9 (Reset) and CN6 pin 7 (INT)
################################################################################
test_fpga_reset_spi_interrupt() {
    print_test_header "TEST 2: FPGA Reset, SPI & Interrupt"
    
    echo "  This test verifies:"
    echo "    - SPI.0 communication (CS0 to CPLD)"
    echo "    - Hardware reset signal"
    echo "    - Interrupt signal functionality"
    
    testmode_begin
    
    # Sub-test 2.1: SPI Write/Read
    echo ""
    echo "  [2.1] Testing SPI write/read to CPLD register 0x04..."
    
    # Write test value 0x42 to LED register 0x04
    spidev_test -D /dev/spidev4.0 -p "\xFF\x04\x42" > /dev/null 2>&1
    
    # Read back the value
    READ1=$(spidev_test -D /dev/spidev4.0 -v -p "\xFE\x04\xFF" 2>&1)
    VAL1=$(get_rx_value "$READ1")
    
    if [ "$VAL1" != "42" ]; then
        testmode_end
        print_result "FAIL" "SPI write/read failed (expected 0x42, got 0x$VAL1)"
        return 1
    fi
    echo "  + SPI write/read OK (value: 0x$VAL1)"
    
    # Sub-test 2.2: Hardware Reset
    echo ""
    echo "  [2.2] Testing hardware reset signal (CN5 pin 9)..."
    echo "    Asserting reset (GPIO low)..."
    gpioset gpiochip1 9=0
    sleep 0.2
    echo "    Releasing reset (GPIO high)..."
    gpioset gpiochip1 9=1
    sleep 0.3
    
    # Read register after reset - should be cleared to 0x00
    READ2=$(spidev_test -D /dev/spidev4.0 -v -p "\xFE\x04\xFF" 2>&1)
    VAL2=$(get_rx_value "$READ2")
    
    if [ "$VAL2" != "00" ]; then
        testmode_end
        print_result "FAIL" "Reset failed (expected 0x00, got 0x$VAL2)"
        return 1
    fi
    echo "  + Reset verified (register cleared to 0x$VAL2)"
    
    # Sub-test 2.3: Interrupt Signal
    echo ""
    echo "  [2.3] Testing interrupt signal (CN6 pin 7)..."
    
    # Check interrupt is high (inactive) after reset
    IRQ1=$(gpioget gpiochip0 1)
    if [ "$IRQ1" != "1" ]; then
        testmode_end
        print_result "FAIL" "Interrupt not high after reset (got $IRQ1)"
        return 1
    fi
    echo "  + Interrupt idle state: high"
    
    # Configure and trigger interrupt
    # Clear interrupt status and mask
    spidev_test -D /dev/spidev4.0 -p "\xFF\x0E\xFF" > /dev/null 2>&1  # Mask all
    spidev_test -D /dev/spidev4.0 -p "\xFF\x09\xFF" > /dev/null 2>&1  # Clear status
    spidev_test -D /dev/spidev4.0 -p "\xFF\x09\x00" > /dev/null 2>&1
    
    # Enable interrupt and set minimum threshold to trigger
    spidev_test -D /dev/spidev4.0 -p "\xFF\x0E\x00" > /dev/null 2>&1  # Enable
    spidev_test -D /dev/spidev4.0 -p "\xFF\x0D\xFF" > /dev/null 2>&1  # Min threshold
    
    # Check interrupt is now low (active)
    IRQ2=$(gpioget gpiochip0 1)
    if [ "$IRQ2" != "0" ]; then
        testmode_end
        print_result "FAIL" "Interrupt trigger failed (expected low, got $IRQ2)"
        return 1
    fi
    echo "  + Interrupt active state: low"
    
    testmode_end
    print_result "PASS" "FPGA Reset, SPI, and Interrupt all verified"
    return 0
}

################################################################################
# TEST 3: SAI Audio Loop Test
# Tests: CN5 pins 2,4,6,7,8 (SAI MCLK, BCLK, SYNC, TX_DATA, RX_DATA)
################################################################################
test_sai_loop() {
    print_test_header "TEST 3: SAI Audio Loop"
    
    echo "  Testing SAI (Serial Audio Interface) data path..."
    echo "  Method: Enable FPGA internal loopback, transmit pattern, verify reception"
    
    DEVICE="hw:0,0"
    TX_FILE="/tmp/sai_test_tx_$$.raw"
    RX_FILE="/tmp/sai_test_rx_$$.raw"
    rm -f "$TX_FILE" "$RX_FILE" 2>/dev/null || true
    
    # Enable FPGA internal SAI loopback
    # This connects TX_DATA directly to RX_DATA inside the FPGA
    echo "  -> Enabling FPGA SAI internal loop..."
    echo 1 > /sys/devices/platform/densitron-audio/fpga_sai_loop
    
    # Create test pattern with incrementing values
    echo "  -> Creating test pattern..."
    python3 -c "
import struct
data = bytearray()
for i in range(2*4*1024):
    data.extend(struct.pack('<I', i)[:4])
with open('$TX_FILE', 'wb') as f:
    f.write(bytes(data))
"
    
    # Start recording and playback simultaneously
    # IMPORTANT: Start arecord BEFORE aplay to ensure recorder is ready
    echo "  -> Running SAI transmit/receive (1 second)..."
    arecord -D "$DEVICE" -t raw -r 48000 -c 2 -f S24_LE \
            --period-size=2048 --buffer-size=8192 -d 1 "$RX_FILE" 2>/dev/null &
    
    aplay -D "$DEVICE" -t raw -r 48000 -c 2 -f S24_LE \
          --period-size=2048 --buffer-size=8192 "$TX_FILE" 2>/dev/null
    
    sleep 0.8
    wait
    
    # Disable loop
    echo 0 > /sys/devices/platform/densitron-audio/fpga_sai_loop
    
    # Verify received data contains transmitted pattern
    echo "  -> Verifying received data..."
    if [ ! -f "$TX_FILE" ] || [ ! -f "$RX_FILE" ]; then
        print_result "FAIL" "Test files not created"
        return 1
    fi
    
    TX_PATTERN=$(hexdump -v -e '1/1 "%02x"' -n 64 "$TX_FILE")
    
    if ( hexdump -v -e '1/1 "%02x"' -n 16384 "$RX_FILE" | grep -q "$TX_PATTERN" ); then
        rm -f "$TX_FILE" "$RX_FILE"
        print_result "PASS" "SAI audio loop verified (pattern match found)"
        return 0
    else
        echo "  DEBUG: TX file size: $(stat -c%s "$TX_FILE" 2>/dev/null || echo 0)"
        echo "  DEBUG: RX file size: $(stat -c%s "$RX_FILE" 2>/dev/null || echo 0)"
        rm -f "$TX_FILE" "$RX_FILE"
        print_result "FAIL" "SAI data mismatch (pattern not found in received data)"
        return 1
    fi
}

################################################################################
# TEST 4: PCM1748 DAC Communication
# Tests: CN6 pins 2,4,6,8,9 (SPI.1 with CS1)
################################################################################
test_pcm1748() {
   
        
    print_test_header "TEST 4: PCM1748 DAC (SPI.1)"
    
    echo "  Testing SPI.1 communication to PCM1748 DAC..."
    echo "  NOTE: PCB REV 1.0 - ZEROR/L signals not connected"
    

    
    ###########################################################################
    ### Test skipped becasue Layout error (no data to CPLD)
    ### Restore in Rev 1.1. Use SPI for inverting signal polarity checking 
    ### the SPI CS.1 signal correctness.

    # Skip test on PCB REV 1.0
    print_result "SKIP" "PCM1748 test skipped (not supported on PCB REV 1.0)"
    return 0
    
    ###########################################################################   
    
    
    DEVICE="hw:0,0"
    TX_FILE="/tmp/pcm_test_tx_$$.raw"
    rm -f "$TX_FILE" 2>/dev/null || true
    
    # Set audio path to headphones (routes through PCM1748)
    echo "  -> Setting audio path to headphones..."
    amixer sset 'Audio Path' 'Headphones' > /dev/null 2>&1
    
    # Create silent audio (all zeros)
    # PCM1748 needs 1048 consecutive zero samples to set flags
    # Using 2048 samples to ensure reliable detection
    echo "  -> Creating zero pattern (2048 samples)..."
    python3 -c "
import struct
data = bytearray()
for i in range(2*4*2048):  # 2048 samples, 2 channels, 4 bytes per sample
    data.extend(struct.pack('<I', 0)[:4])
with open('$TX_FILE', 'wb') as f:
    f.write(bytes(data))
"
    
    # Play zeros to DAC
    echo "  -> Playing zeros to PCM1748..."
    aplay -D "$DEVICE" -t raw -r 48000 -c 2 -f S24_LE \
          --period-size=2048 --buffer-size=8192 "$TX_FILE" 2>/dev/null
    
    sleep 0.5
    
    # Check zero detection flags
    # PCM1748 sets ZEROR/L high after detecting 1048 consecutive zero samples
    # Expected: 0x03 (R_ZERO=1 L_ZERO=1)
    echo "  -> Reading zero detection status..."
    STATUS=$(cat /sys/devices/platform/densitron-audio/pcm1748_zero_status 2>/dev/null || echo "0x00")
    
    rm -f "$TX_FILE"
    
    # Extract hex value (0x03 or 0x3)
    HEX_VAL=$(echo "$STATUS" | grep -oE '0x[0-9A-Fa-f]+' | head -1)
    
    if [ "$HEX_VAL" = "0x03" ] || [ "$HEX_VAL" = "0x3" ]; then
        print_result "PASS" "PCM1748 communication OK (zero flags: $STATUS)"
        return 0
    else
        print_result "FAIL" "PCM1748 test failed (expected 0x03, got: $STATUS)"
        return 1
    fi
}

################################################################################
# Main Test Execution
################################################################################

echo "+====================================================================+"
echo "|            CN5/CN6 CONNECTOR TEST SUITE                            |"
echo "|                                                                    |"
echo "|  Tests all critical connections between CPU and Audio boards       |"
echo "+====================================================================+"

# Run all tests
test_i2c
test_fpga_reset_spi_interrupt
test_sai_loop
test_pcm1748

# Print summary
echo ""
echo "======================================================================="
echo "                         TEST SUMMARY"
echo "======================================================================="
echo "  Tests Passed: $TESTS_PASSED / $TESTS_TOTAL"
echo "  Tests Failed: $TESTS_FAILED / $TESTS_TOTAL"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo "** ALL TESTS PASSED"
    echo "======================================================================="
    exit 0
else
    echo "!! SOME TESTS FAILED"
    echo "======================================================================="
    exit 1
fi
