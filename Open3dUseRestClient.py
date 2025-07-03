import requests
import numpy as np
import open3d as o3d
import time
import threading
from queue import Queue
import math

class NeatoXV11API:
    def __init__(self, api_url='http://localhost:5000/api/lidar'):
        self.api_url = api_url
        self.data_queue = Queue()
        self.running = True

    def continuous_scan(self):
        while self.running:
            try:
                response = requests.get(self.api_url)
                if response.status_code == 200:
                    data = response.json()
                    if data['is_full_scan']:
                        scan_data = data['scan_data']
                        # Muunnetaan data samaan muotoon kuin alkuperäisessä ohjelmassa
                        formatted_scan = []
                        for point in scan_data:
                            formatted_scan.append({
                                'angle': point['angle'],
                                'distance': point['distance'],
                                'signal': point['signal'],
                                'invalid': point['invalid']
                            })
                        self.data_queue.put(formatted_scan)
                time.sleep(0.1)  # Rajapyyntöjen välinen viive
            except requests.exceptions.RequestException as e:
                print(f"API request failed: {e}")
                time.sleep(1)

    def stop(self):
        self.running = False

class Visualizer:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.pcd = o3d.geometry.PointCloud()
        self.first_run = True
        # Zoom settings
        self.ctr = self.vis.get_view_control()
        self.ctr.set_constant_z_far(100000)
        self.ctr.set_constant_z_near(0.001)
        # Dark theme
        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])
        
        # Create distance circles
        self.distance_circles = []
        self._create_distance_circles()
        
    def _create_distance_circle(self, radius, color, segments=100):
        circle = o3d.geometry.LineSet()
        points = []
        lines = []
        
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append([x, y, 0])
            
            if i > 0:
                lines.append([i-1, i])
        # Close the circle
        lines.append([segments-1, 0])
        
        circle.points = o3d.utility.Vector3dVector(points)
        circle.lines = o3d.utility.Vector2iVector(lines)
        circle.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
        
        return circle
    
    def _create_distance_circles(self):
        max_distance = 7
        dark_gray = [0.2, 0.2, 0.2]
        purple = [0.6, 0.2, 0.6]
        
        # Create set of radii for purple circles for faster lookups
        purple_radii = {round(r, 1) for r in np.arange(0.0, max_distance + 0.1, 1.0)}
        
        # Create all circles
        for r in np.arange(0.1, max_distance + 0.01, 0.1):
            rounded_r = round(r, 1)  # Avoid floating point precision issues
            if rounded_r in purple_radii:
                # Add purple circle (skip gray)
                circle = self._create_distance_circle(r, purple)
            else:
                # Add gray circle
                circle = self._create_distance_circle(r, dark_gray)
            self.distance_circles.append(circle)

    def update_visualization(self, scan):
        valid = [p for p in scan if not p['invalid'] and p['distance'] > 0]
        if not valid:
            return
            
        # Oikea kulmalaskenta: käännä vain 90 astetta myötäpäivään
        angles = np.radians([-p['angle'] + 90 for p in valid])  # Muutettu miinusmerkki ja -90°
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
        
        # Create direction line (blue) - osoittaa ylös (0, 0.2, 0)
        line_points = np.array([[0, 0, 0], [0, 0.2, 0]])  # 20 cm ylöspäin
        line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(line_points),
            lines=o3d.utility.Vector2iVector([[0, 1]]),
        )
        line.colors = o3d.utility.Vector3dVector([[0, 0, 1]])  # Blue
        
        if self.first_run:
            self.vis.add_geometry(self.pcd)
            self.vis.add_geometry(origin_point)
            self.vis.add_geometry(line)
            # Add distance circles
            for circle in self.distance_circles:
                self.vis.add_geometry(circle)
            self.first_run = False
        else:
            self.vis.update_geometry(self.pcd)
            self.vis.update_geometry(origin_point)
            self.vis.update_geometry(line)
            # Update distance circles
            for circle in self.distance_circles:
                self.vis.update_geometry(circle)
            
        self.vis.poll_events()
        self.vis.update_renderer()

if __name__ == "__main__":
    print("🔄 Starting real-time visualization using REST API...")
    
    lidar = NeatoXV11API()  # Oletusosoite on localhost:5000/api/lidar
    visualizer = Visualizer()
    
    # Start scanning in a thread
    scan_thread = threading.Thread(target=lidar.continuous_scan, daemon=True)
    scan_thread.start()
    
    try:
        while True:
            if not lidar.data_queue.empty():
                scan = lidar.data_queue.get()
                print(f"✅ Visualizing {len([p for p in scan if not p['invalid']])} valid points")
                visualizer.update_visualization(scan)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n💀 Closing program...")
        lidar.stop()
        visualizer.vis.destroy_window()