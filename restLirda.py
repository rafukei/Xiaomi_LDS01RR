from flask import Flask, jsonify
import serial
import struct
from time import time
from threading import Thread, Lock
import time as system_time
from collections import defaultdict

app = Flask(__name__)

class NeatoXV11:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.packet_length = 22
        self.motor_rpm = 0
        self.scan_data = {}  # Tallennetaan kulmat dictionaryyn
        self.last_scan_time = 0
        self.lock = Lock()
        self.running = True
        self.full_scan_ready = False

    def _check_crc(self, packet):
        chk32 = 0
        for i in range(0, 20, 2):
            word = packet[i] + (packet[i+1] << 8)
            chk32 = (chk32 << 1) + word
        
        checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
        checksum &= 0x7FFF
        crc_l = packet[20]
        crc_m = packet[21]
        
        return (checksum & 0xFF) == crc_l and (checksum >> 8) == crc_m

    def _parse_packet(self, packet):
        if not self._check_crc(packet):
            print("Virhe: Väärä CRC")
            return None

        index = packet[1] - 0xA0
        start_angle = index * 4

        speed = (packet[3] << 8) + packet[2]
        self.motor_rpm = speed / 64.0

        points = []
        for i in range(4):
            offset = 4 + i * 4
            dist_lsb = packet[offset]
            flags_dist_msb = packet[offset + 1]
            signal_lsb = packet[offset + 2]
            signal_msb = packet[offset + 3]

            invalid_data = (flags_dist_msb & 0x80) != 0
            strength_warning = (flags_dist_msb & 0x40) != 0

            distance = 0
            if not invalid_data:
                distance = dist_lsb + ((flags_dist_msb & 0x3F) << 8)

            signal = signal_lsb + (signal_msb << 8)
            angle = (start_angle + i) % 360

            points.append({
                'angle': angle,
                'distance': distance,
                'signal': signal,
                'invalid': invalid_data,
                'strength_warning': strength_warning
            })

        return points

    def read_scan(self):
        buffer = bytearray()
        while self.running:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                continue
            
            buffer.extend(data)
            
            while len(buffer) >= self.packet_length:
                start_pos = buffer.find(0xFA)
                if start_pos == -1:
                    buffer.clear()
                    break
                
                buffer = buffer[start_pos:]
                
                if len(buffer) < self.packet_length:
                    break
                    
                packet = buffer[:self.packet_length]
                buffer = buffer[self.packet_length:]
                
                points = self._parse_packet(packet)
                if not points:
                    continue

                with self.lock:
                    for point in points:
                        angle = point['angle']
                        self.scan_data[angle] = point  # Päivitä kulma

                    # Tarkista onko täysi skannaus (360 astetta)
                    if len(self.scan_data) == 360:
                        self.full_scan_ready = True
                        self.last_scan_time = system_time.time()

    def get_full_scan(self):
        with self.lock:
            # Luo lista 360 arvosta (täytä puuttuvat edellisillä arvoilla)
            full_scan = []
            for angle in range(360):
                if angle in self.scan_data:
                    full_scan.append(self.scan_data[angle])
                else:
                    # Jos kulmaa ei ole, palauta tyhjä arvo
                    full_scan.append({
                        'angle': angle,
                        'distance': 0,
                        'signal': 0,
                        'invalid': True,
                        'strength_warning': False
                    })
            return {
                'timestamp': self.last_scan_time,
                'motor_rpm': self.motor_rpm,
                'scan_data': full_scan,
                'is_full_scan': self.full_scan_ready
            }

    def stop(self):
        self.running = False

# Alusta LIDAR ja käynnistä säie
lidar = NeatoXV11('/dev/ttyUSB1')
lidar_thread = Thread(target=lidar.read_scan)
lidar_thread.daemon = True
lidar_thread.start()

@app.route('/api/lidar', methods=['GET'])
def get_lidar_data():
    scan_data = lidar.get_full_scan()
    return jsonify(scan_data)

if __name__ == "__main__":
    try:
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        lidar.stop()
        lidar_thread.join()
        print("\nLopetetaan.")