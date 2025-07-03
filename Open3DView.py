import serial
import struct
import numpy as np
import open3d as o3d
import time
import threading
from queue import Queue

class NeatoXV11:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.packet_length = 22
        self.scan_data = []
        self.data_queue = Queue()

    def _check_crc(self, packet):
        chk32 = 0
        for i in range(0, 20, 2):
            word = packet[i] + (packet[i + 1] << 8)
            chk32 = (chk32 << 1) + word
        checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
        checksum &= 0x7FFF
        crc_l = packet[20]
        crc_m = packet[21]
        return (checksum & 0xFF) == crc_l and (checksum >> 8) == crc_m

    def _parse_packet(self, packet):
        if not self._check_crc(packet):
            return None

        index = packet[1] - 0xA0
        start_angle = index * 4
        speed = (packet[3] << 8) + packet[2]

        points = []
        for i in range(4):
            offset = 4 + i * 4
            dist_lsb = packet[offset]
            flags_dist_msb = packet[offset + 1]
            signal_lsb = packet[offset + 2]
            signal_msb = packet[offset + 3]

            invalid_data = (flags_dist_msb & 0x80) != 0
            distance = 0
            if not invalid_data:
                distance = dist_lsb + ((flags_dist_msb & 0x3F) << 8)
            signal = signal_lsb + (signal_msb << 8)
            angle = (start_angle + i) % 360

            points.append({
                'angle': angle,
                'distance': distance,
                'signal': signal,
                'invalid': invalid_data
            })

        return points

    def continuous_scan(self):
        buffer = bytearray()
        current_scan = []
        
        while True:
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

                for p in points:
                    if p['angle'] == 0 and current_scan:
                        self.data_queue.put(current_scan)
                        current_scan = []
                    current_scan.append(p)

class Visualizer:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.pcd = o3d.geometry.PointCloud()
        self.first_run = True
        # Zoom settings
        # Get camera settings and remove zoom limits
        self.ctr = self.vis.get_view_control()
        self.ctr.set_constant_z_far(100000)  # Set very large value (e.g. 100km)
        self.ctr.set_constant_z_near(0.001)   # Smallest possible near distance
        # Dark theme
        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])  # Dark gray
    def update_visualization(self, scan):
        valid = [p for p in scan if not p['invalid'] and p['distance'] > 0]
        if not valid:
            return
            
        angles = np.radians([p['angle'] for p in valid])
        distances = np.array([p['distance'] for p in valid]) / 1000.0  # mm -> m
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        z = np.zeros_like(x)

        points = np.vstack((x, y, z)).T
        
        # Create point cloud (white points)
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd.colors = o3d.utility.Vector3dVector(np.ones((len(points), 3)))  # White
        
        # Create origin point (red point)
        origin_point = o3d.geometry.PointCloud()
        origin_point.points = o3d.utility.Vector3dVector([[0, 0, 0]])
        origin_point.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # Red
        
        # Create direction line (blue)
        line_points = np.array([[0, 0, 0], [0.2, 0, 0]])  # 20 cm line
        line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(line_points),
            lines=o3d.utility.Vector2iVector([[0, 1]]),
        )
        line.colors = o3d.utility.Vector3dVector([[0, 0, 1]])  # Blue
        
        if self.first_run:
            self.vis.add_geometry(self.pcd)
            self.vis.add_geometry(origin_point)
            self.vis.add_geometry(line)
            self.first_run = False
        else:
            self.vis.update_geometry(self.pcd)
            self.vis.update_geometry(origin_point)  # Also update the point
            
        self.vis.poll_events()
        self.vis.update_renderer()

if __name__ == "__main__":
    print("🔄 Starting real-time visualization...")
    
    lidar = NeatoXV11('/dev/ttyUSB0')  # Change port if needed
    visualizer = Visualizer()
    
    # Start scanning in a thread
    scan_thread = threading.Thread(target=lidar.continuous_scan, daemon=True)
    scan_thread.start()
    
    try:
        while True:
            if not lidar.data_queue.empty():
                scan = lidar.data_queue.get()
                print(f"✅ Visualizing {len(scan)} points")
                visualizer.update_visualization(scan)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n💀 Closing program...")
        visualizer.vis.destroy_window()