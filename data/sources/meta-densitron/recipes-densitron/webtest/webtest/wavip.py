import struct
import math

def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
    return crc & 0xFF

def create_waveform(ip, filename="ipaddr.wav", samplerate=48000, symbol_time=0.5,
                   base_freq=10000, training_freq=1500, training_time=1.0):
    
    octets = [int(x) for x in ip.split('.')]
    crc = crc8(octets)
    data = octets + [crc]
    
    samples = []
    
    # Training signal
    for i in range(int(samplerate * training_time)):
        t = i / samplerate
        samples.append(int(32767 * 0.8 * math.sin(2 * math.pi * training_freq * t)))
    
    # Data
    for byte_val in data:
        freq = base_freq + byte_val * 10
        for i in range(int(samplerate * symbol_time)):
            t = i / samplerate
            samples.append(int(32767 * 0.8 * math.sin(2 * math.pi * freq * t)))
    
    # Manual WAV file creation
    with open(filename, 'wb') as f:
        # WAV header
        f.write(b'RIFF')
        f.write(struct.pack('<I', 36 + len(samples) * 2))
        f.write(b'WAVE')
        f.write(b'fmt ')
        f.write(struct.pack('<IHHIIHH', 16, 1, 1, samplerate, samplerate * 2, 2, 16))
        f.write(b'data')
        f.write(struct.pack('<I', len(samples) * 2))
        
        # Audio data
        for sample in samples:
            f.write(struct.pack('<h', sample))
    
    print(f"Created: {filename} ({len(samples)/samplerate:.2f}s)")

if __name__ == "__main__":
    create_waveform("192.168.1.100", "ipaddr.wav")
