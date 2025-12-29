#!/usr/bin/env python3
"""
Audio Interface Web Server - ALSA Implementation
Version 2.2.1  : added wav iP for automatic connection on running the server
Version 2.2.2  : added html color repreentation for LEDs. Added Headset connection status
Version 2.2.4  : python3 app_backend.py
"""

from flask import Flask, render_template, jsonify, request, redirect, send_file, Response
import subprocess
import time
import os
import sys
import argparse
import json
import signal
import re
import glob
import threading
import logging
import socket
import atexit
import zipfile
import shutil
from collections import defaultdict, deque
import csv
import uuid

# Virtual input device reading
try:
    import evdev
    from evdev import InputDevice, categorize, ecodes
    EVDEV_AVAILABLE = True
except ImportError:
    print("WARNING: python-evdev not available")
    EVDEV_AVAILABLE = False

# Import wavip module
import wavip

APP_VERSION = "2.2.5"

SESSION_ID = str(uuid.uuid4())

audio_process = None
record_process = None
playback_process = None
wavip_process = None  # Process for playing IP address audio
gui_process = None  # GTK GUI process
loop_test_process = None  # Audio loop test process

# Rotary encoder state - 2 rotaries with buttons
rotary_state = {
    "rotary1": {"position": 0, "last_delta": 0, "button_pressed": False},
    "rotary2": {"position": 0, "last_delta": 0, "button_pressed": False}
}
rotary_lock = threading.Lock()

# Button event state
button_events = deque(maxlen=8)  # Keep last 8 events
button_events_lock = threading.Lock()

# Button coordinate mapping (system coordinates from newcoordinates.csv)
# UI Button Visual Positions (from app_ui.ui)
UI_BUTTON_POSITIONS = {
    '10d': (121, 148),
    '10l': (212, 172),
    '10r': (31, 172),
    '10u': (121, 201),
    '11d': (121, 0),      # Missing from UI, add center estimate
    '11l': (212, 29),
    '11r': (31, 29),
    '11u': (121, 53),
    '12d': (121, 362),
    '12l': (31, 336),
    '12r': (212, 336),
    '12u': (121, 309),
    '13d': (121, 510),
    '13l': (31, 486),
    '13r': (212, 486),
    '13u': (121, 457),
    '14d': (436, 362),
    '14l': (346, 336),
    '14r': (527, 336),
    '14u': (436, 309),
    '15d': (436, 510),
    '15l': (346, 486),
    '15r': (527, 486),
    '15u': (436, 457),
    '16d': (749, 362),
    '16l': (659, 336),
    '16r': (840, 336),
    '16u': (749, 309),
    '17d': (749, 510),
    '17l': (659, 486),
    '17r': (840, 486),
    '17u': (749, 457),
    '18d': (1064, 362),
    '18l': (974, 336),
    '18r': (1155, 336),
    '18u': (1064, 309),
    '19d': (1064, 510),
    '19l': (974, 486),
    '19r': (1155, 486),
    '19u': (1064, 457),
    '4d': (1064, 148),
    '4l': (1155, 172),
    '4r': (974, 172),
    '4u': (1064, 201),
    '5d': (1064, 0),      
    '5l': (1155, 29),
    '5r': (974, 29),
    '5u': (1064, 53), 
    '6d': (749, 148),
    '6l': (840, 172),
    '6r': (659, 172),
    '6u': (749, 201),
    '7d': (749, 0),       
    '7l': (840, 29),
    '7r': (659, 29),
    '7u': (749, 53),
    '8d': (436, 148),
    '8l': (527, 172),
    '8r': (346, 172),
    '8u': (436, 201),
    '9d': (436, 0),      
    '9l': (527, 29),
    '9r': (346, 29),
    '9u': (436, 53),
    'l1': (1270, 205),
    'l2': (1270, 99),
    'l3': (1270, 0),      
    'r1': (1270, 285),
    'r2': (1270, 391),
    'r3': (1270, 490),
}

# Touch Sensor Coordinates (keep this for button detection)
TOUCH_SENSOR_COORDS = {
    '4l': (13107, 714), '4u': (14687, 3222), '4d': (11001, 3222), '4r': (13107, 5937),
    '5l': (4330, 714), '5u': (6027, 3222), '5d': (2224, 3222), '5r': (4330, 5937),
    '6l': (13107, 7962), '6u': (14687, 10470), '6d': (11001, 10470), '6r': (13107, 13117),
    '7l': (4330, 7962), '7u': (6027, 10470), '7d': (2224, 10470), '7r': (4330, 13117),
    '8l': (13107, 15187), '8u': (14687, 17673), '8d': (11001, 17673), '8r': (13107, 20365),
    '9l': (4330, 15187), '9u': (6027, 17673), '9d': (2224, 17673), '9r': (4330, 20365),
    '10l': (13107, 22390), '10u': (14687, 24921), '10d': (11001, 24921), '10r': (13107, 27544),
    '11l': (4330, 22390), '11u': (6027, 24921), '11d': (2224, 24921), '11r': (4330, 27544),
    '12l': (20714, 27544), '12u': (18607, 24921), '12d': (22411, 24921), '12r': (20714, 22390),
    '13l': (29491, 27544), '13u': (27384, 24921), '13d': (31071, 24921), '13r': (29491, 22390),
    '14l': (20714, 20365), '14u': (18607, 17673), '14d': (22411, 17673), '14r': (20714, 15187),
    '15l': (29491, 20365), '15u': (27384, 17673), '15d': (31071, 17673), '15r': (29491, 15187),
    '16l': (20714, 13117), '16u': (18607, 10470), '16d': (22411, 10470), '16r': (20714, 7962),
    '17l': (29491, 13117), '17u': (27384, 10470), '17d': (31071, 10470), '17r': (29491, 7962),
    '18l': (20714, 5937), '18u': (18607, 3222), '18d': (22411, 3222), '18r': (20714, 714),
    '19l': (29491, 5937), '19u': (27384, 3222), '19d': (31071, 3222), '19r': (29491, 714),
    'l1': (19076, 30605), 'l2': (24927, 30605), 'l3': (30837, 30605),
    'r1': (14453, 30605), 'r2': (8543, 30605), 'r3': (2692, 30605)
}

LARGE_BUTTONS = ['l1', 'l2', 'l3', 'r1', 'r2', 'r3']

# ============================================================================
# Coordinate Transformation Functions
# ============================================================================

def get_button_size(button_id):
    """Get button size based on button ID"""
    if button_id in LARGE_BUTTONS:
        return (130, 70)
    return (50, 50)

def ui_to_physical(ui_x, ui_y, button_id):
    """
    Convert UI top-left coordinates to physical top-left coordinates
    
    Args:
        ui_x, ui_y: Top-left corner in UI space
        button_id: Button identifier to determine size
    
    Returns:
        (phys_x, phys_y): Top-left corner in physical space
    """
    width, height = get_button_size(button_id)
    
    if ui_y < 280:
        # Upper half LEFT display (rotated -270°)
        # Transformation flips axes, so adjust for button size
        phys_x = 1424 - ui_x - width
        phys_y = 280 - ui_y - height
    else:
        # Lower half RIGHT display (no rotation)
        phys_x = ui_x + 1424
        phys_y = ui_y - 280
    
    return (phys_x, phys_y)

def touch_to_system(touch_x, touch_y):
    """
    Convert touch sensor coordinates (32767-32767) to system space (1424-560)
    Used only for touch detection, not for display
    """
    scaled_x = int(touch_x * 1424 / 32767)
    scaled_y = int(touch_y * 560 / 32767)
    return (scaled_x, scaled_y)

def find_button_by_hash(button_hash):
    """Find button ID from hash value"""
    for button_id in UI_BUTTON_POSITIONS.keys():  # Changed from BUTTON_COORDS
        if hash(button_id) % 256 == button_hash:
            return button_id
    return None
    


# ============================================================================
# IP Address and WAVIP Functions
# ============================================================================

def get_external_ip():
    """Get the external IP address visible from network"""
    try:
        # Create a socket to determine the external IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # Connect to Google DNS
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print(f"Error getting external IP: {e}")
        return "127.0.0.1"

def start_wavip_playback():
    """Start playing wavip.wav in a loop with 1 second delay"""
    global wavip_process
    try:
        # Stop any existing wavip playback
        if wavip_process is not None and wavip_process.poll() is None:
            os.killpg(os.getpgid(wavip_process.pid), signal.SIGTERM)
        
        # Play in loop: while true; do aplay wavip.wav; sleep 1; done
        cmd = "while true; do aplay wavip.wav; sleep 1; done"
        wavip_process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True
        )
        print(f"Started wavip playback (PID: {wavip_process.pid})")
    except Exception as e:
        print(f"Error starting wavip playback: {e}")

def stop_wavip_playback():
    """Stop wavip playback"""
    global wavip_process
    try:
        if wavip_process is not None and wavip_process.poll() is None:
            os.killpg(os.getpgid(wavip_process.pid), signal.SIGTERM)
            wavip_process = None
            print("Stopped wavip playback")
            return True
        return False
    except Exception as e:
        print(f"Error stopping wavip playback: {e}")
        return False

def cleanup_all_processes():
    """Kill all running subprocesses on exit"""
    global audio_process, record_process, playback_process, wavip_process, gui_process, loop_test_process
    
    print("\nCleaning up processes...")
    
    # Kill audio_process
    if audio_process is not None and audio_process.poll() is None:
        try:
            os.killpg(os.getpgid(audio_process.pid), signal.SIGTERM)
            print("Stopped audio process")
        except Exception as e:
            print(f"Error stopping audio process: {e}")
    
    # Kill record_process
    if record_process is not None and record_process.poll() is None:
        try:
            os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
            print("Stopped record process")
        except Exception as e:
            print(f"Error stopping record process: {e}")
    
    # Kill playback_process
    if playback_process is not None and playback_process.poll() is None:
        try:
            os.killpg(os.getpgid(playback_process.pid), signal.SIGTERM)
            print("Stopped playback process")
        except Exception as e:
            print(f"Error stopping playback process: {e}")
    
    # Kill wavip_process
    if wavip_process is not None and wavip_process.poll() is None:
        try:
            os.killpg(os.getpgid(wavip_process.pid), signal.SIGTERM)
            print("Stopped wavip process")
        except Exception as e:
            print(f"Error stopping wavip process: {e}")
    
    # Kill GUI process
    if gui_process is not None and gui_process.poll() is None:
        try:
            gui_process.terminate()
            gui_process.wait(timeout=5)
            print("Stopped GUI process")
        except Exception as e:
            print(f"Error stopping GUI process: {e}")
    # Kill loop test process
    if loop_test_process is not None and loop_test_process.poll() is None:
        try:
            os.killpg(os.getpgid(loop_test_process.pid), signal.SIGTERM)
            print("Stopped loop test process")
        except Exception as e:
            print(f"Error stopping loop test process: {e}")
    
    print("Cleanup complete")

def signal_handler(sig, frame):
    """Handle termination signals"""
    print(f"\nReceived signal {sig}")
    cleanup_all_processes()
    sys.exit(0)


# ============================================================================
# GUI and Button Event Functions
# ============================================================================

def start_gui_application():
    """Start the GTK GUI application"""
    global gui_process
    try:
        gui_script = os.path.join(os.path.dirname(__file__), 'app_backend.py')
        if os.path.exists(gui_script):
            gui_process = subprocess.Popen(
                ['python3', '-u', gui_script],
                stdout=None,  # Allow output to console
                stderr=None   # Allow errors to console
            )
            print(f"Started GUI application (PID: {gui_process.pid})")
            return True
        else:
            print(f"GUI script not found: {gui_script}")
            return False
    except Exception as e:
        print(f"Error starting GUI application: {e}")
        return False

def button_event_reader():
    """Read button events from file written by app_backend.py"""
    global button_events, button_events_lock, button_state_register, button_state_lock
    
    event_log_file = '/tmp/button_events.json'
    seen_events = set()  # Track events by timestamp to avoid duplicates
    
    print("Button event reader: Started (reading from file)")
    
    while True:
        try:
            # Check if file exists
            if not os.path.exists(event_log_file):
                time.sleep(0.05)
                continue
            
            # Read events from file
            with open(event_log_file, 'r') as f:
                events = json.load(f)
            
            # Process only unseen events
            new_events = []
            for evt in events:
                event_key = f"{evt['timestamp']}_{evt['button_id']}_{evt['event_type']}"
                if event_key not in seen_events:
                    seen_events.add(event_key)
                    
                    # Add physical coordinates
                    if evt['button_id'] in UI_BUTTON_POSITIONS:
                        ui_x, ui_y = UI_BUTTON_POSITIONS[evt['button_id']]
                        phys_x, phys_y = ui_to_physical(ui_x, ui_y, evt['button_id'])
                        evt['physical_x'] = phys_x
                        evt['physical_y'] = phys_y
                    else:
                        evt['physical_x'] = 'N/A'
                        evt['physical_y'] = 'N/A'
                    
                    # Update state register
                    with button_state_lock:
                        if evt['event_type'] == 'press':
                            button_state_register[evt['button_id']] = True
                        elif evt['event_type'] == 'release':
                            button_state_register[evt['button_id']] = False
                    
                    new_events.append(evt)
            
            if new_events:
                with button_events_lock:
                    # Add new events to the front (newest first)
                    button_events.extendleft(new_events)
                    
                    # Keep only last 8 events
                    while len(button_events) > 8:
                        button_events.pop()
            
            # Clean up old seen events (keep last 1000)
            if len(seen_events) > 1000:
                seen_events.clear()
            
        except json.JSONDecodeError:
            # File being written, try again
            pass
        except Exception as e:
            print(f"Error reading button events: {e}")
        
        time.sleep(0.1)  # Poll every 100ms 


# ============================================================================
# SYSFS Access Functions
# ============================================================================

SYSFS_BASE = "/sys/devices/platform/densitron-audio"
LED_BASE = "/sys/class/leds"

def sysfs_read(attribute):
    """Read sysfs attribute value"""
    try:
        path = f"{SYSFS_BASE}/{attribute}"
        if os.path.exists(path):
            with open(path, 'r') as f:
                return f.read().strip()
        return None
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading {attribute}: {e}")
        return None

def sysfs_write(attribute, value):
    """Write sysfs attribute value"""
    try:
        path = f"{SYSFS_BASE}/{attribute}"
        if os.path.exists(path):
            with open(path, 'w') as f:
                f.write(str(value))
            return True
        return False
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error writing {attribute}: {e}")
        return False

def led_set_brightness(led_num, color, brightness):
    """Set LED brightness via LED subsystem"""
    try:
        path = f"{LED_BASE}/densitron:{color}:led{led_num}/brightness"
        if os.path.exists(path):
            with open(path, 'w') as f:
                f.write(str(brightness))
            return True
        return False
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error setting LED{led_num} {color}: {e}")
        return False

def led_get_brightness(led_num, color):
    """Get LED brightness via LED subsystem"""
    try:
        path = f"{LED_BASE}/densitron:{color}:led{led_num}/brightness"
        if os.path.exists(path):
            with open(path, 'r') as f:
                return int(f.read().strip())
        return 0
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading LED{led_num} {color}: {e}")
        return 0

def list_sysfs_attributes():
    """List all available sysfs attributes"""
    try:
        if not os.path.exists(SYSFS_BASE):
            return []
        attrs = []
        for item in os.listdir(SYSFS_BASE):
            path = os.path.join(SYSFS_BASE, item)
            if os.path.isfile(path):
                attrs.append(item)
        return sorted(attrs)
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error listing attributes: {e}")
        return []

def get_rotary_position():
    """Read rotary switch position from sysfs"""
    try:
        pos_str = sysfs_read("rotary_position")
        if pos_str:
            # Parse format: "Left:   1; Right:  -1"
            result = {"raw": pos_str, "left": 0, "right": 0}
            
            # Extract left value
            left_match = re.search(r'Left:\s*(-?\d+)', pos_str)
            if left_match:
                result["left"] = int(left_match.group(1))
            
            # Extract right value
            right_match = re.search(r'Right:\s*(-?\d+)', pos_str)
            if right_match:
                result["right"] = int(right_match.group(1))
            
            return result
        return {"raw": "N/A", "left": 0, "right": 0}
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading rotary position: {e}")
        return {"raw": "error", "left": 0, "right": 0}

def get_headset_status():
    """Read headset detection status from sysfs"""
    try:
        # Read headphones_connected attribute
        status = sysfs_read("headphones_connected")
        if status is not None:
            return {"plugged": status == "1", "method": "sysfs", "value": status}
        
        return {"plugged": False, "method": "unknown", "value": "N/A"}
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading headset status: {e}")
        return {"plugged": False, "method": "error", "value": str(e)}


# ============================================================================
# Rotary Encoder Reading
# ============================================================================

parser = argparse.ArgumentParser(description='Audio Interface Web Server')
parser.add_argument('-d', '--debug', action='store_true', help='Enable debug mode')
args = parser.parse_args()

app = Flask(__name__)
DEBUG_MODE = args.debug

ALSA_CARD = "0"

# Audio files directory
AUDIO_DIR = os.path.dirname(os.path.abspath(sys.argv[0])) + "/sound"
TESTIF_DIR = os.path.dirname(os.path.abspath(sys.argv[0])) 

# Volume/gain limits - ALSA uses percentage 0-100
SPEAKER_VOLUME_MAX = 100
HEADPHONE_VOLUME_MAX = 100
MIC_GAIN_MAX = 100


# ============================================================================
# Version Information
# ============================================================================

def get_fpga_version():
    """Read FPGA version from sysfs"""
    try:
        paths = [
            "/sys/devices/platform/densitron-audio/fpga_fw_version"
        ]
        for path in paths:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    return f.read().strip()
        
        # Try dmesg for FPGA version
        result = subprocess.run(['cat', path], capture_output=True, text=True, timeout=2)
        if result.returncode == 0 and result.stdout:
            return result.stdout.strip()
            
        return "Not detected"
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading FPGA version: {e}")
        return "Not detected"


def get_dts_model():
    """Read device tree model"""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            return f.read().strip().rstrip('\x00')
    except Exception:
        return "Unknown"


def get_driver_version():
    """Get snd-soc-inf1000 driver version"""
    try:
        # Try multiple driver names
        driver = ['snd-soc-inf1000']
        result = subprocess.run(['modinfo', '-F', 'version', 'snd-soc-inf1000'], capture_output=True, text=True, timeout=2)
        if result.returncode == 0 and result.stdout:
            return result.stdout.strip()
        
        # Check loaded modules
        with open('/proc/modules', 'r') as f:
            for line in f:
                if 'inf1000' in line.lower() or 'densitron' in line.lower():
                    parts = line.split()
                    if len(parts) > 0:
                        return f"Loaded ({parts[0]})"
        
        return "Not detected"
    except Exception:
        return "Not detected"


# ============================================================================
# ALSA Control Functions
# ============================================================================

def get_alsa_control(control_name):
    """Get ALSA control value (returns percentage 0-100)"""
    try:
        # Use amixer to get percentage value
        cmd = ['amixer', '-c', ALSA_CARD, 'sget', control_name]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        
        if result.returncode == 0:
            # Parse percentage like [75%]
            match = re.search(r'\[(\d+)%\]', result.stdout)
            if match:
                return int(match.group(1))
        return None
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading {control_name}: {e}")
        return None


def set_alsa_control(control_name, value):
    """Set ALSA control value (accepts percentage 0-100)"""
    try:
        # Use amixer sset with percentage
        cmd = ['amixer', '-c', ALSA_CARD, 'sset', control_name, f'{value}%']
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        return result.returncode == 0
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error setting {control_name}: {e}")
        return False


def get_alsa_enum(control_name):
    """Get ALSA enum control value"""
    try:
        cmd = ['amixer', '-c', ALSA_CARD, 'sget', control_name]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        
        if result.returncode == 0:
            # Parse Item0: 'Speaker' or similar
            match = re.search(r"Item\d+:\s*'([^']+)'", result.stdout)
            if match:
                return match.group(1)
        return None
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading enum {control_name}: {e}")
        return None


def set_alsa_enum(control_name, value):
    """Set ALSA enum control value"""
    try:
        cmd = ['amixer', '-c', ALSA_CARD, 'sset', control_name, value]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        return result.returncode == 0
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error setting enum {control_name}: {e}")
        return False


# ============================================================================
# Audio Control
# ============================================================================

def get_all_audio_settings():
    """Read all audio settings from ALSA"""
    try:
        return {
            "speaker_left_volume": get_alsa_control("Speaker Left") or 0,
            "speaker_right_volume": get_alsa_control("Speaker Right") or 0,
            "hp_left_volume": get_alsa_control("Headphone Left") or 0,
            "hp_right_volume": get_alsa_control("Headphone Right") or 0,
            "mic_headset_gain": get_alsa_control("Headset Mic") or 0,
            "mic_front_gain": get_alsa_control("Front Mic") or 0,
            "audio_path": get_alsa_enum("Audio Path") or "Speakers",
            "mic_source": get_alsa_enum("Input Source") or "Front",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading audio settings: {e}")
        return {
            "speaker_left_volume": 0, "speaker_right_volume": 0,
            "hp_left_volume": 0, "hp_right_volume": 0,
            "mic_headset_gain": 0, "mic_front_gain": 0,
            "audio_path": "Speakers", "mic_source": "Front",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "error": str(e)
        }


def set_speaker_volume(channel, value):
    """Set speaker volume via ALSA"""
    try:
        value = max(0, min(SPEAKER_VOLUME_MAX, int(value)))
        control = f"Speaker {channel.capitalize()}"
        success = set_alsa_control(control, value)
        return {"success": success, "channel": channel, "value": value}
    except Exception as e:
        return {"success": False, "error": str(e)}


def set_headphone_volume(channel, value):
    """Set headphone volume via ALSA"""
    try:
        value = max(0, min(HEADPHONE_VOLUME_MAX, int(value)))
        control = f"Headphone {channel.capitalize()}"
        success = set_alsa_control(control, value)
        return {"success": success, "channel": channel, "value": value}
    except Exception as e:
        return {"success": False, "error": str(e)}


def set_mic_gain(mic_type, value):
    """Set microphone gain via ALSA"""
    try:
        value = max(0, min(MIC_GAIN_MAX, int(value)))
        control = f"{mic_type} Mic"
        success = set_alsa_control(control, value)
        return {"success": success, "mic_type": mic_type, "value": value}
    except Exception as e:
        return {"success": False, "error": str(e)}


def set_audio_path(path):
    """Set audio output path (Speakers/Headphones)"""
    try:
        if path not in ["Speakers", "Headphones"]:
            return {"success": False, "error": "Invalid path"}
        success = set_alsa_enum("Audio Path", path)
        return {"success": success, "path": path}
    except Exception as e:
        return {"success": False, "error": str(e)}


def set_mic_source(source):
    """Set microphone source (Front/Headset/Both/None)"""
    try:
        valid = ["None", "Front", "Headset", "Both"]
        if source not in valid:
            return {"success": False, "error": f"Invalid source. Must be: {', '.join(valid)}"}
                    
        success = set_alsa_enum("Input Source", source)
        return {"success": success, "source": source}
    except Exception as e:
        return {"success": False, "error": str(e)}


# ============================================================================
# LED Control
# ============================================================================

def write_rgb_led(led_number, r, g, b):
    """Write RGB values to LED"""
    try:
        r = max(0, min(255, int(r)))
        g = max(0, min(255, int(g)))
        b = max(0, min(255, int(b)))
        
        led_prefix = f"densitron:*:led{led_number}"
        
        for color, value in [('red', r), ('green', g), ('blue', b)]:
            leds = glob.glob(f"/sys/class/leds/{led_prefix.replace('*', color)}")
            if leds:
                with open(f"{leds[0]}/brightness", 'w') as f:
                    f.write(str(value))
        
        try:
            if os.path.exists('/tmp/led_rgb_values.json'):
                with open('/tmp/led_rgb_values.json', 'r') as f:
                    led_data = json.loads(f.read())
            else:
                led_data = {'led1': {'r': 0, 'g': 0, 'b': 0}, 'led2': {'r': 0, 'g': 0, 'b': 0}}
            
            led_data[f'led{led_number}'] = {'r': r, 'g': g, 'b': b}
            
            with open('/tmp/led_rgb_values.json', 'w') as f:
                f.write(json.dumps(led_data))
        except Exception:
            pass
        
        return {"success": True, "led": led_number, "rgb": {"r": r, "g": g, "b": b}}
    except Exception as e:
        return {"success": False, "error": str(e)}


def read_led_values():
    """Read LED RGB values"""
    led1_rgb = {'r': 0, 'g': 0, 'b': 0}
    led2_rgb = {'r': 0, 'g': 0, 'b': 0}
    
    try:
        if os.path.exists('/tmp/led_rgb_values.json'):
            with open('/tmp/led_rgb_values.json', 'r') as f:
                led_data = json.loads(f.read())
                led1_rgb = led_data.get('led1', led1_rgb)
                led2_rgb = led_data.get('led2', led2_rgb)
    except Exception:
        pass
    
    return led1_rgb, led2_rgb


# ============================================================================
# Rotary Encoder / Buttons (evdev)
# ============================================================================

def init_rotary_encoder():
    """Initialize rotary encoder reading threads - finds Densitron devices dynamically"""
    try:
        import evdev
        
        # Find Densitron rotary devices by name
        densitron_rotaries = []
        for path in evdev.list_devices():
            device = evdev.InputDevice(path)
            if 'Densitron' in device.name and 'Rotary' in device.name:
                densitron_rotaries.append(device)
        
        # Sort by name to ensure consistent left=1, right=2
        densitron_rotaries.sort(key=lambda d: d.name)
        
        if len(densitron_rotaries) < 2:
            if DEBUG_MODE:
                print(f"Warning: Found only {len(densitron_rotaries)} Densitron rotary device(s)")
            return
        
        # Start thread for each device
        for idx, device in enumerate(densitron_rotaries[:2]):
            rotary_num = idx + 1
            thread = threading.Thread(
                target=read_rotary_events, 
                args=(device, rotary_num), 
                daemon=True
            )
            thread.start()
            if DEBUG_MODE:
                print(f"Rotary {rotary_num}: {device.name} ({device.path})")
                
    except ImportError:
        if DEBUG_MODE:
            print("evdev not installed. Install with: pip3 install evdev")
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error initializing rotary encoders: {e}")


def read_rotary_events(device, rotary_num):
    """Read events from rotary encoder device"""
    try:
        import evdev
        
        rotary_key = f"rotary{rotary_num}"
        
        for event in device.read_loop():
            if event.type == evdev.ecodes.EV_REL:
                # Handle rotation (REL_WHEEL for Densitron encoders)
                if event.code in (evdev.ecodes.REL_WHEEL, evdev.ecodes.REL_X, evdev.ecodes.REL_DIAL):
                    with rotary_lock:
                        rotary_state[rotary_key]["last_delta"] = event.value
                        rotary_state[rotary_key]["position"] += event.value
                        
            elif event.type == evdev.ecodes.EV_KEY:
                # Handle button press
                with rotary_lock:
                    rotary_state[rotary_key]["button_pressed"] = (event.value == 1)
                    
    except Exception as e:
        if DEBUG_MODE:
            print(f"Error reading rotary {rotary_num}: {e}")


def get_rotary_state():
    """Get current rotary encoder states with position, last delta, and button state"""
    with rotary_lock:
        return {
            "rotary1": {
                "position": rotary_state["rotary1"]["position"],
                "last_delta": rotary_state["rotary1"]["last_delta"],
                "button_pressed": rotary_state["rotary1"]["button_pressed"]
            },
            "rotary2": {
                "position": rotary_state["rotary2"]["position"],
                "last_delta": rotary_state["rotary2"]["last_delta"],
                "button_pressed": rotary_state["rotary2"]["button_pressed"]
            }
        }
        

# ============================================================================
# Flask Routes
# ============================================================================

@app.route('/')
def index():
    """Main page"""
    audio_settings = get_all_audio_settings()
    rotary = get_rotary_state()
    led1_rgb, led2_rgb = read_led_values()
    
    data = build_data_dict(audio_settings, rotary, led1_rgb, led2_rgb)
    return render_template('index.html', data=data, debug_mode=DEBUG_MODE)
    return render_template('index.html')


@app.route('/api/read')
def api_read():
    audio_settings = get_all_audio_settings()
    rotary = get_rotary_state()
    led1_rgb, led2_rgb = read_led_values()
    
    data = build_data_dict(audio_settings, rotary, led1_rgb, led2_rgb)
    return jsonify(data)


@app.route('/api/version')
def api_version():
    """Get version information"""
    return jsonify({
        "app_version": APP_VERSION,
        "fpga_version": get_fpga_version(),
        "dts_model": get_dts_model(),
        "driver_version": get_driver_version()
    })


def build_data_dict(audio_settings, rotary, led1_rgb, led2_rgb):
    """Build data dictionary for frontend"""
    return {
        **audio_settings,
        # Rotary 1 + Button 1 + LED 1
        "rotary1_position": rotary["rotary1"]["position"],
        "button1_pressed": rotary["rotary1"]["button_pressed"],
        # Rotary 2 + Button 2 + LED 2
        "rotary2_position": rotary["rotary2"]["position"],
        "button2_pressed": rotary["rotary2"]["button_pressed"],
        # LED data
        "led1_rgb": led1_rgb,
        "led2_rgb": led2_rgb,
        "led1_hex": f"#{led1_rgb['r']:02x}{led1_rgb['g']:02x}{led1_rgb['b']:02x}",
        "led2_hex": f"#{led2_rgb['r']:02x}{led2_rgb['g']:02x}{led2_rgb['b']:02x}",
        "is_audio_playing": audio_process is not None and audio_process.poll() is None,
        "is_loop_test_running": loop_test_process is not None and loop_test_process.poll() is None,
        "version_info": {
            "app": APP_VERSION,
            "fpga": get_fpga_version(),
            "dts": get_dts_model(),
            "driver": get_driver_version()
        },
        # Legacy compatibility - keep for existing interface
        "register1": {
            "success": True, 
            "register_name": "Audio Path Config",
            "register_value": 1 if audio_settings.get("audio_path") == "Headphone" else 0,
            "register_value_hex": "0x01" if audio_settings.get("audio_path") == "Headphone" else "0x00",
            "description": "Audio output routing: 0=Speaker, 1=Headphone",
            "tx_data": "", "rx_data": "", "raw_output": f"Audio Path: {audio_settings.get('audio_path', 'Unknown')}"
        },
        "register2": {
            "success": True,
            "register_name": "Mic Source Config", 
            "register_value": 1 if audio_settings.get("mic_source") == "Headset" else 0,
            "register_value_hex": "0x01" if audio_settings.get("mic_source") == "Headset" else "0x00",
            "description": "Microphone input selection: 0=Front Mic, 1=Headset Mic",
            "tx_data": "", "rx_data": "", "raw_output": f"Mic Source: {audio_settings.get('mic_source', 'Unknown')}"
        },
        "register3": {
            "success": True,
            "register_name": "Rotary Status",
            "register_value": (rotary["rotary1"]["position"] & 0xFF),
            "register_value_hex": f"0x{rotary['rotary1']['position'] & 0xFF:02X}",
            "description": f"Rotary1: {rotary['rotary1']['position']}, Btn1: {'PRESSED' if rotary['rotary1']['button_pressed'] else 'Released'}, Rotary2: {rotary['rotary2']['position']}, Btn2: {'PRESSED' if rotary['rotary2']['button_pressed'] else 'Released'}",
            "tx_data": "", "rx_data": "", "raw_output": ""
        },
        "headphones_connected": audio_settings.get("audio_path") == "Headphone",
        "left_volume": audio_settings.get("speaker_left_volume", 0),
        "right_volume": audio_settings.get("speaker_right_volume", 0),
        "mic_left_gain": audio_settings.get("mic_front_gain", 0),
        "mic_right_gain": audio_settings.get("mic_headset_gain", 0),
        "left_mic_gain": {"success": True, "register_value": audio_settings.get("mic_front_gain", 0), "register_value_hex": f"0x{audio_settings.get('mic_front_gain', 0):02X}"},
        "right_mic_gain": {"success": True, "register_value": audio_settings.get("mic_headset_gain", 0), "register_value_hex": f"0x{audio_settings.get('mic_headset_gain', 0):02X}"},
        "audio_playing_file": None,
        "is_mic_test_active": False,
        "mic_test_type": None
    }


@app.route('/api/set_volume', methods=['POST'])
def api_set_volume():
    try:
        data = request.get_json()
        result = set_speaker_volume(data.get('channel'), data.get('value'))
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/set_headphone_volume', methods=['POST'])
def api_set_headphone_volume():
    try:
        data = request.get_json()
        result = set_headphone_volume(data.get('channel'), data.get('value'))
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/set_microphone_gain', methods=['POST'])
def api_set_microphone_gain():
    try:
        data = request.get_json()
        channel = data.get('channel')
        mic_type = "Front" if channel == "left" else "Headset"
        result = set_mic_gain(mic_type, data.get('value'))
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/set_audio_path', methods=['POST'])
def api_set_audio_path():
    try:
        data = request.get_json()
        result = set_audio_path(data.get('path'))
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/set_mic_source', methods=['POST'])
def api_set_mic_source():
    try:
        data = request.get_json()
        result = set_mic_source(data.get('source'))
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/set_rgb_led', methods=['POST'])
def api_set_rgb_led():
    try:
        data = request.get_json()
        result = write_rgb_led(data.get('led'), data.get('r'), data.get('g'), data.get('b'))
        return jsonify(result)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/play_audio_freq', methods=['POST'])
def api_play_audio_freq():
    """Play audio test signal"""
    global audio_process
    
    try:
        data = request.get_json()
        action = data.get('action')
        
        if action == 'start':
            # Stop any existing playback
            if audio_process is not None and audio_process.poll() is None:
                os.killpg(os.getpgid(audio_process.pid), signal.SIGTERM)
            
            frequency = data.get('frequency', 1000)
            duration = data.get('duration', 5)
            
            # Play tone using speaker-test
            cmd = f"speaker-test -t sine -f {frequency} -l 1 -c 2"
            audio_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            
            # Stop after duration
            def stop_after(duration):
                time.sleep(duration)
                if audio_process and audio_process.poll() is None:
                    os.killpg(os.getpgid(audio_process.pid), signal.SIGTERM)
            
            threading.Thread(target=stop_after, args=(duration,), daemon=True).start()
            
            return jsonify({"success": True, "status": "playing"})
            
        elif action == 'stop':
            if audio_process is not None and audio_process.poll() is None:
                os.killpg(os.getpgid(audio_process.pid), signal.SIGTERM)
                audio_process = None
                return jsonify({"success": True, "status": "stopped"})
            else:
                return jsonify({"success": False, "error": "Not playing"})
                
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/play_status')
def api_play_status():
    """Check if audio is playing"""
    global audio_process
    is_playing = audio_process is not None and audio_process.poll() is None
    return jsonify({"is_playing": is_playing})
@app.route('/api/loop_test', methods=['POST'])
def api_loop_test():
    """Start/stop audio loop test (arecord -> aplay)"""
    global loop_test_process
    
    try:
        data = request.get_json()
        action = data.get('action')
        
        if action == 'start':
            # Stop any existing loop test
            if loop_test_process is not None and loop_test_process.poll() is None:
                os.killpg(os.getpgid(loop_test_process.pid), signal.SIGTERM)
                loop_test_process = None
            
            # Start loop test: arecord | aplay
            arecord_cmd = ['arecord', '-r', '48000', '-c', '2', '--period-size=64', '-f', 'S24_LE']
            aplay_cmd = ['aplay', '-r', '48000', '-f', 'S24_LE']
            
            # Create pipeline
            arecord_proc = subprocess.Popen(
                arecord_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            
            aplay_proc = subprocess.Popen(
                aplay_cmd,
                stdin=arecord_proc.stdout,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            
            # Store the arecord process (killing it will stop the pipeline)
            loop_test_process = arecord_proc
            
            return jsonify({"success": True, "message": "Loop test started"})
            
        elif action == 'stop':
            if loop_test_process is not None and loop_test_process.poll() is None:
                os.killpg(os.getpgid(loop_test_process.pid), signal.SIGTERM)
                loop_test_process = None
                return jsonify({"success": True, "message": "Loop test stopped"})
            else:
                return jsonify({"success": True, "message": "Loop test not running"})
        
        else:
            return jsonify({"success": False, "error": "Invalid action"})
            
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/loop_test_status')
def api_loop_test_status():
    """Check if loop test is running"""
    global loop_test_process
    is_running = loop_test_process is not None and loop_test_process.poll() is None
    return jsonify({"is_running": is_running})


@app.route('/api/connector_test', methods=['POST'])
def api_connector_test():
    """Run comprehensive connector tests using unified script"""
    connector_test_script = os.path.join(TESTIF_DIR, "connector_test.sh")
    try:
        # Run unified test script with increased timeout
        with open("/tmp/connector_test.log", "w") as log_file:
            result = subprocess.run(
                ['/bin/bash', str(connector_test_script)],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                timeout=45,
                text=True
            )
    
        print(f"run:  {TESTIF_DIR + '/connector_test.sh'}")
        
        # Give script time to complete and flush output
        time.sleep(1)
        
        # Read output from log file
        with open("/tmp/connector_test.log", "r") as f:
            output = f.read()
            
        print(f"read:  '/tmp/connector_test.log'")
        if DEBUG_MODE:
            print(output)
        
        # Parse each test result from script output
        tests = []
        test_patterns = [
            ("I2C Communication (TAS5733L)", "TEST 1"),
            ("FPGA Reset, SPI & Interrupt", "TEST 2"),
            ("SAI Audio Loop", "TEST 3"),
            ("PCM1748 DAC (SPI.1)", "TEST 4")
        ]
        
        for name, test_marker in test_patterns:
            # Find test section
            if test_marker in output:
                # Find where this test starts
                test_start = output.find(test_marker)
                # Find where next test starts (or end of output)
                next_test_idx = len(output)
                for next_name, next_marker in test_patterns:
                    if next_marker != test_marker:
                        next_idx = output.find(next_marker, test_start + len(test_marker))
                        if next_idx > test_start and next_idx < next_test_idx:
                            next_test_idx = next_idx
                
                # Extract just this test's section
                test_section = output[test_start:next_test_idx]
                
                if "** PASS" in test_section:
                    status = "PASS"
                    # Extract detail from PASS line
                    for line in test_section.split('\n'):
                        if "** PASS" in line:
                            detail = line.replace("** PASS - ", "").replace("** PASS", "").strip()
                            break
                    else:
                        detail = "Test completed successfully"
                elif "!! FAIL" in test_section and "skipped" in test_section.lower():
                    status = "SKIP"
                    # Extract detail from FAIL line with skip
                    for line in test_section.split('\n'):
                        if "!! FAIL" in line:
                            detail = line.replace("!! FAIL - ", "").replace("!! FAIL", "").strip()
                            break
                    else:
                        detail = "Test skipped"
                elif "!! FAIL" in test_section:
                    status = "FAIL"
                    for line in test_section.split('\n'):
                        if "!! FAIL" in line:
                            detail = line.replace("!! FAIL - ", "").replace("!! FAIL", "").strip()
                            break
                    else:
                        detail = "Test failed - check logs"
                else:
                    status = "UNKNOWN"
                    detail = "Could not find test result marker"
            else:
                status = "UNKNOWN"
                detail = "Test marker not found in output"
            
            tests.append({
                "name": name,
                "status": status,
                "detail": detail
            })
        
        # Determine overall result
        all_passed = result.returncode == 0
        
        return jsonify({
            "success": True,
            "overall": "PASS" if all_passed else "FAIL",
            "tests": tests,
            "full_output": output
        })
        
    except subprocess.TimeoutExpired:
        return jsonify({
            "success": False, 
            "error": "Test timeout - tests took longer than 45 seconds"
        })
    except Exception as e:
        import traceback
        return jsonify({
            "success": False, 
            "error": str(e),
            "traceback": traceback.format_exc()
        })


@app.route('/api/list_audio_files')
def api_list_audio_files():
    """List all .wav files in the audio directory"""
    try:
        if not os.path.exists(AUDIO_DIR):
            os.makedirs(AUDIO_DIR)
        
        files = [f for f in os.listdir(AUDIO_DIR) if f.endswith('.wav')]
        files.sort()
        return jsonify({"success": True, "files": files})
    except Exception as e:
        return jsonify({"success": False, "error": str(e), "files": []})


@app.route('/api/record_audio', methods=['POST'])
def api_record_audio():
    """Record audio using arecord"""
    global record_process
    
    try:
        data = request.get_json()
        action = data.get('action')
        
        if action == 'start':
            # Stop any existing recording
            if record_process is not None and record_process.poll() is None:
                os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
            
            filename = data.get('filename', 'recording.wav')
            if not filename.endswith('.wav'):
                filename += '.wav'
            
            filepath = os.path.join(AUDIO_DIR, filename)
            
            # Ensure directory exists
            if not os.path.exists(AUDIO_DIR):
                os.makedirs(AUDIO_DIR)
            
            # Start recording: arecord -D hw:0,0 -f cd -t wav filename.wav
            cmd = f"arecord -f cd -t wav '{filepath}'"
            print("Running shell command:", cmd)
            record_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            return jsonify({"success": True, "status": "recording", "filename": filename})
            
        elif action == 'stop':
            if record_process is not None and record_process.poll() is None:
                os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
                record_process = None
                return jsonify({"success": True, "status": "stopped"})
            else:
                return jsonify({"success": False, "error": "No recording in progress"})
                
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/play_audio_file', methods=['POST'])
def api_play_audio_file():
    """Play audio file using aplay"""
    global playback_process
    
    try:
        data = request.get_json()
        action = data.get('action')
        
        if action == 'start':
            # Stop any existing playback
            if playback_process is not None and playback_process.poll() is None:
                os.killpg(os.getpgid(playback_process.pid), signal.SIGTERM)
            
            filename = data.get('filename')
            if not filename:
                return jsonify({"success": False, "error": "No filename provided"})
            
            filepath = os.path.join(AUDIO_DIR, filename)
            
            if not os.path.exists(filepath):
                return jsonify({"success": False, "error": "File not found"})
            
            # Start playback: aplay -D hw:0,0 filename.wav
            cmd = f"aplay '{filepath}'"            
            playback_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            return jsonify({"success": True, "status": "playing", "filename": filename})
            
        elif action == 'stop':
            if playback_process is not None and playback_process.poll() is None:
                os.killpg(os.getpgid(playback_process.pid), signal.SIGTERM)
                playback_process = None
                return jsonify({"success": True, "status": "stopped"})
            else:
                return jsonify({"success": False, "error": "No playback in progress"})
                
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/playback_status')
def api_playback_status():
    """Check if playback is still active"""
    global playback_process
    is_playing = playback_process is not None and playback_process.poll() is None
    return jsonify({"is_playing": is_playing})


@app.route('/api/rotary_status')
def api_rotary_status():
    """Get current rotary encoder positions, last deltas, and button states"""
    global rotary_state
    try:
        with rotary_lock:
            # Prepare response with current state from evdev
            response_data = {
                "rotary1": {
                    "position": rotary_state["rotary1"]["position"],
                    "last_delta": rotary_state["rotary1"]["last_delta"],
                    "button_pressed": rotary_state["rotary1"]["button_pressed"]
                },
                "rotary2": {
                    "position": rotary_state["rotary2"]["position"],
                    "last_delta": rotary_state["rotary2"]["last_delta"],
                    "button_pressed": rotary_state["rotary2"]["button_pressed"]
                }
            }
        
        return jsonify({
            "success": True,
            "encoders": response_data,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e),
            "encoders": {
                "rotary1": {"position": 0, "last_delta": 0, "button_pressed": False},
                "rotary2": {"position": 0, "last_delta": 0, "button_pressed": False}
            }
        })


@app.route('/api/upload_audio_file', methods=['POST'])
def api_upload_audio_file():
    """Upload audio file"""
    try:
        if 'file' not in request.files:
            return jsonify({"success": False, "error": "No file provided"})
        
        file = request.files['file']
        if file.filename == '':
            return jsonify({"success": False, "error": "No file selected"})
        
        if not file.filename.lower().endswith('.wav'):
            return jsonify({"success": False, "error": "Only WAV files allowed"})
        
        # Ensure directory exists
        if not os.path.exists(AUDIO_DIR):
            os.makedirs(AUDIO_DIR)
        
        # Save file
        filepath = os.path.join(AUDIO_DIR, file.filename)
        file.save(filepath)
        
        return jsonify({"success": True, "filename": file.filename})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/download_audio_file')
def api_download_audio_file():
    """Download audio file"""
    try:
        filename = request.args.get('filename')
        if not filename:
            return jsonify({"success": False, "error": "No filename provided"}), 400
        
        filepath = os.path.join(AUDIO_DIR, filename)
        if not os.path.exists(filepath):
            return jsonify({"success": False, "error": "File not found"}), 404
        
        return send_file(filepath, as_attachment=True, download_name=filename)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


# ============================================================================
# Audio Files API 
# ============================================================================

@app.route('/api/delete_audio_file', methods=['POST'])
def api_delete_audio_file():
    """Delete audio file"""
    try:
        data = request.get_json()
        filename = data.get('filename')
        
        if not filename:
            return jsonify({"success": False, "error": "No filename provided"})
        
        filepath = os.path.join(AUDIO_DIR, filename)
        if not os.path.exists(filepath):
            return jsonify({"success": False, "error": "File not found"})
        
        os.remove(filepath)
        return jsonify({"success": True, "message": f"Deleted {filename}"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/backup_testif', methods=['POST'])
def api_backup_testif():
    """Create zip backup of testif directory"""
    try:
        if not os.path.exists(TESTIF_DIR):
            return jsonify({"success": False, "error": "testif directory not found"})
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        zip_filename = f"testif_backup_{timestamp}.zip"
        zip_path = os.path.join(TESTIF_DIR, zip_filename)
        
        # Create zip file
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, dirs, files in os.walk(TESTIF_DIR):
                for file in files:
                    if file != zip_filename:  # Don't include the zip itself
                        file_path = os.path.join(root, file)
                        arcname = os.path.relpath(file_path, TESTIF_DIR)
                        zipf.write(file_path, arcname)
        
        return jsonify({
            "success": True,
            "filename": zip_filename,
            "message": f"Created backup: {zip_filename}"
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/download_backup')
def api_download_backup():
    """Download testif backup zip file"""
    try:
        filename = request.args.get('filename')
        if not filename:
            return jsonify({"success": False, "error": "No filename provided"}), 400
        
        filepath = os.path.join(TESTIF_DIR, filename)
        if not os.path.exists(filepath):
            return jsonify({"success": False, "error": "File not found"}), 404
        
        return send_file(filepath, as_attachment=True, download_name=filename)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

# ============================================================================
# Register Debug API Routes (SYSFS)
# ============================================================================

@app.route('/api/registers/list')
def api_list_registers():
    """List all available sysfs attributes"""
    try:
        attrs = list_sysfs_attributes()
        return jsonify({"success": True, "attributes": attrs})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

@app.route('/api/registers/read', methods=['POST'])
def api_read_register():
    """Read sysfs attribute value"""
    try:
        data = request.get_json()
        attribute = data.get('attribute', 'fpga_fw_version')
        
        value = sysfs_read(attribute)
        
        if value is not None:
            return jsonify({
                "success": True,
                "value": value,
                "attribute": attribute
            })
        else:
            return jsonify({"success": False, "error": "Attribute not found or not readable"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/registers/write', methods=['POST'])
def api_write_register():
    """Write sysfs attribute value"""
    try:
        data = request.get_json()
        attribute = data.get('attribute', '')
        value = data.get('value', '')
        
        success = sysfs_write(attribute, value)
        
        if success:
            return jsonify({"success": True, "message": "Attribute written successfully"})
        else:
            return jsonify({"success": False, "error": "Failed to write attribute"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# ============================================================================
# Headset Detection API
# ============================================================================

@app.route('/api/headset_status')
def api_headset_status():
    """Get headset detection status"""
    try:
        status = get_headset_status()
        return jsonify({"success": True, **status})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/headset_threshold', methods=['GET', 'POST'])
def api_headset_threshold():
    """Get or set headset detection threshold via sysfs (if available)"""
    try:
        # Check if attribute exists
        threshold_attrs = ['headset_threshold', 'headphone_threshold', 'jack_threshold']
        attr_found = None
        
        for attr in threshold_attrs:
            test_val = sysfs_read(attr)
            if test_val is not None:
                attr_found = attr
                break
        
        if not attr_found:
            return jsonify({
                "success": False, 
                "error": "Threshold control not available in hardware",
                "note": "Your hardware may not expose this setting"
            })
        
        if request.method == 'POST':
            data = request.get_json()
            threshold = data.get('threshold', 128)
            
            success = sysfs_write(attr_found, threshold)
            
            if success:
                return jsonify({"success": True, "threshold": threshold, "attribute": attr_found})
            else:
                return jsonify({"success": False, "error": "Failed to set threshold"})
        else:
            # Read current threshold
            threshold_str = sysfs_read(attr_found)
            if threshold_str is not None:
                try:
                    threshold = int(threshold_str)
                    return jsonify({"success": True, "threshold": threshold, "attribute": attr_found})
                except ValueError:
                    return jsonify({"success": False, "error": f"Invalid threshold value: {threshold_str}"})
            else:
                return jsonify({"success": False, "error": "Failed to read threshold"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# ============================================================================
# LED Color Control API
# ============================================================================

@app.route('/api/led_color', methods=['GET', 'POST'])
def api_led_color():
    """Get or set LED RGB color via LED subsystem"""
    try:
        # LED number (1 or 2)
        led_num = request.args.get('led', '1') if request.method == 'GET' else request.get_json().get('led', '1')
        
        if request.method == 'POST':
            data = request.get_json()
            color = data.get('color', '#000000')
            
            # Parse hex color #RRGGBB
            r = int(color[1:3], 16)
            g = int(color[3:5], 16)
            b = int(color[5:7], 16)
            
            # Write to LED subsystem
            success_r = led_set_brightness(led_num, 'red', r)
            success_g = led_set_brightness(led_num, 'green', g)
            success_b = led_set_brightness(led_num, 'blue', b)
            
            if success_r and success_g and success_b:
                return jsonify({"success": True, "led": led_num, "color": color, "r": r, "g": g, "b": b})
            else:
                return jsonify({"success": False, "error": "Failed to set LED color"})
        else:
            # Read current color
            r = led_get_brightness(led_num, 'red')
            g = led_get_brightness(led_num, 'green')
            b = led_get_brightness(led_num, 'blue')
            color = f"#{r:02x}{g:02x}{b:02x}"
            
            return jsonify({"success": True, "led": led_num, "color": color, "r": r, "g": g, "b": b})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# ============================================================================
# Button Events API
# ============================================================================
# Global button state register
button_state_register = {}
button_state_lock = threading.Lock()


# One-time initialization flag
initialization_done = False
initialization_lock = threading.Lock()

@app.route('/api/button_states')
def api_button_states():
    """Get current state of all buttons"""
    global initialization_done
    
    try:
        # One-time initialization on first API call
        with initialization_lock:
            if not initialization_done:
                # Clear stale data
                event_log_file = '/tmp/button_events.json'
                if os.path.exists(event_log_file):
                    os.remove(event_log_file)
                
                # Initialize all buttons to unpressed
                with button_state_lock:
                    for button_id in UI_BUTTON_POSITIONS.keys():
                        button_state_register[button_id] = False
                
                with button_events_lock:
                    button_events.clear()
                
                print("First API call - cleared stale data and initialized button states")
                initialization_done = True
        
        with button_state_lock:
            return jsonify({"success": True, "states": button_state_register.copy()})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})
        
@app.route('/api/button_events')
def api_button_events():
    """Get recent button events"""
    global initialization_done
    
    try:
        # One-time initialization on first API call
        with initialization_lock:
            if not initialization_done:
                # Clear stale data
                event_log_file = '/tmp/button_events.json'
                if os.path.exists(event_log_file):
                    os.remove(event_log_file)
                
                # Initialize all buttons to unpressed
                with button_state_lock:
                    for button_id in UI_BUTTON_POSITIONS.keys():
                        button_state_register[button_id] = False
                
                with button_events_lock:
                    button_events.clear()
                
                print("First API call - cleared stale data and initialized button states")
                initialization_done = True
        
        with button_events_lock:
            events_list = list(button_events)
        return jsonify({"success": True, "events": events_list})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

@app.route('/api/button_stream')
def button_stream():
    """Server-Sent Events stream for button events"""
    def event_stream():
        last_count = 0
        while True:
            with button_events_lock:
                current_count = len(button_events)
                if current_count > last_count:
                    # New events available
                    events_list = list(button_events)
                    yield f"data: {json.dumps(events_list)}\n\n"
                    last_count = current_count
            time.sleep(0.1)
    
    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/api/button_coordinates')
def api_button_coordinates():
    """Get all button coordinates in physical display space"""
    try:
        physical_coords = {}        
        print(f"physical_coords: {physical_coords}")
        for button_id, (ui_x, ui_y) in UI_BUTTON_POSITIONS.items():
            phys_x, phys_y = ui_to_physical(ui_x, ui_y, button_id)
            physical_coords[button_id] = {
                'ui': {'x': ui_x, 'y': ui_y},
                'physical': {'x': phys_x, 'y': phys_y}
            }
        return jsonify({"success": True, "buttons": physical_coords})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

@app.route('/display_monitor.html')
def display_monitor():
    """Serve the full view display monitor page"""
    return render_template('display_monitor.html')
    
@app.route('/api/clear_button_events', methods=['POST'])
def api_clear_button_events():
    """Clear all button events and reset state register"""
    try:
        global button_events, button_state_register
        
        with button_events_lock:
            button_events.clear()
        
        with button_state_lock:
            button_state_register.clear()
        
        event_log_file = '/tmp/button_events.json'
        if os.path.exists(event_log_file):
            with open(event_log_file, 'w') as f:
                json.dump([], f)
        
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})
        
@app.route('/api/session_id')
def api_session_id():
    """Return current session ID"""
    return jsonify({"success": True, "session_id": SESSION_ID})

# ============================================================================
# WAVIP Endpoint
# ============================================================================

@app.route('/stopwavip')
def stopwavip():
    """Stop the wavip playback when accessed"""
    stop_wavip_playback()
    return redirect('/')
    
@app.route('/api/wavip/status')
def api_wavip_status():
    """Check if wavip autoplay is enabled"""
    flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.wavip_autoplay')
    enabled = os.path.exists(flag_file)
    return jsonify({"success": True, "enabled": enabled})

@app.route('/api/wavip/set', methods=['POST'])
def api_wavip_set():
    """Enable or disable wavip autoplay"""
    try:
        data = request.get_json()
        enabled = data.get('enabled', False)
        flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.wavip_autoplay')
        
        if enabled:
            with open(flag_file, 'w') as f:
                f.write('enabled')
        else:
            if os.path.exists(flag_file):
                os.remove(flag_file)
        
        return jsonify({"success": True, "enabled": enabled})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})
        
# ============================================================================
# GUI Animation Toggle Button
# ============================================================================
@app.route('/api/gui_animation/status')
def api_gui_animation_status():
    """Check if GUI animation is enabled"""
    flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.playguianimation')
    enabled = os.path.exists(flag_file)
    return jsonify({"success": True, "enabled": enabled})

@app.route('/api/gui_animation/toggle', methods=['POST'])
def api_gui_animation_toggle():
    """Toggle GUI animation flag file"""
    try:
        flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.playguianimation')
        
        if os.path.exists(flag_file):
            os.remove(flag_file)
            enabled = False
        else:
            with open(flag_file, 'w') as f:
                f.write('enabled')
            enabled = True
        
        return jsonify({"success": True, "enabled": enabled})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

# ============================================================================
# Play IP at startup / autostart
# ============================================================================
@app.route('/api/autostart/status')
def api_autostart_status():
    """Check if autostart is enabled"""
    flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.autostart')
    enabled = os.path.exists(flag_file)
    return jsonify({"success": True, "enabled": enabled})

@app.route('/api/autostart/set', methods=['POST'])
def api_autostart_set():
    """Enable or disable autostart"""
    try:
        data = request.get_json()
        enabled = data.get('enabled', False)
        flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.autostart')
        
        if enabled:
            with open(flag_file, 'w') as f:
                f.write('enabled')
        else:
            if os.path.exists(flag_file):
                os.remove(flag_file)
        
        return jsonify({"success": True, "enabled": enabled})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})
        
# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == '__main__':
    # Register signal handlers for cleanup
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(cleanup_all_processes)

    print("=" * 70)
    print(f"Audio Interface Web Server v{APP_VERSION}")
    print("=" * 70)
    print(f"Mode:                {'DEBUG' if DEBUG_MODE else 'NORMAL'}")
    print(f"ALSA Card:           {ALSA_CARD}")
    print(f"FPGA Version:        {get_fpga_version()}")
    print(f"DTS Model:           {get_dts_model()}")
    print(f"Driver Version:      {get_driver_version()}")
    print(f"Speaker Volume Max:  {SPEAKER_VOLUME_MAX}")
    print(f"Headphone Vol Max:   {HEADPHONE_VOLUME_MAX}")
    print(f"Mic Gain Max:        {MIC_GAIN_MAX}")
    print("=" * 70)
    
    # Get external IP address
    external_ip = get_external_ip()
    print(f"External IP:         {external_ip}")
    
    # Generate wavip.wav file
    print("Generating wavip.wav...")
    wavip.create_waveform(external_ip, "wavip.wav")
    
    # Start wavip playback only if flag exists
    flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.wavip_autoplay')
    if os.path.exists(flag_file):
        print("Starting wavip playback loop (autoplay enabled)...")
        start_wavip_playback()
    else:
        print("Wavip autoplay disabled (flag not found)")
    
    print("=" * 70)
    print(f"Server running at:   http://0.0.0.0:5000")
    print(f"Stop WAVIP at:       http://{external_ip}:5000/stopwavip")
    print("=" * 70)
    
    # Start GUI application
    print("Starting GUI application...")
    if start_gui_application():
        print("GUI application started successfully")
    else:
        print("WARNING: GUI application failed to start")
        
    # Initialize default audio settings
    print("Setting default audio configuration for tranmitting IP by speakers")
    set_audio_path("Speakers")
    set_speaker_volume("left", 90)
    set_speaker_volume("right", 90)
    print("Default audio: Speakers at 90%")
    
    # Start button event reader thread
    if EVDEV_AVAILABLE:
        event_thread = threading.Thread(target=button_event_reader, daemon=True)
        event_thread.start()
        print("Button event reader thread started")
    else:
        print("WARNING: Button event monitoring not available (python-evdev required)")
        
    # Initialize button state register (all buttons unpressed)
    with button_state_lock:
        for button_id in UI_BUTTON_POSITIONS.keys():
            button_state_register[button_id] = False
    print(f"Initialized {len(button_state_register)} buttons to unpressed state")

    # Clear stale button events from previous session
    event_log_file = '/tmp/button_events.json'
    if os.path.exists(event_log_file):
        os.remove(event_log_file)
        print("Cleared stale button events file")
    
    
    init_rotary_encoder()
    
    # Disable Flask's default request logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # Only show errors, not requests
    
    app.run(host='0.0.0.0', port=5000, debug=False)
