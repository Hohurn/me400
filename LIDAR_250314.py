

import asyncio
import serial_asyncio
import math
import struct
from dataclasses import dataclass
from typing import List, Optional
import numpy as np
import matplotlib.pyplot as plt


# --------------------------------------------------------------------
# Constants & CRC Table
# --------------------------------------------------------------------
HEADER_VALUE = 0x54
LENGTH_VALUE = 0x2C  # 0x2C (44 in decimal) represents the payload length
PACKET_SIZE = 47     # Total packet size: 1+1+2+2+36+2+2+1 = 47 bytes


# Full CRC table with 256 values (must match LiDAR device's algorithm)
CRC_TABLE = [
       0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
]


# --------------------------------------------------------------------
# Data structure for a LiDAR point
# --------------------------------------------------------------------
@dataclass
class PointData:
   angle: float      # Angle in degrees
   distance: int     # Raw distance (e.g., in mm)
   confidence: int   # Confidence value


# --------------------------------------------------------------------
# LiDAR Packet Processing Class
# --------------------------------------------------------------------
class LiPkg:
   def __init__(self):
       self.timestamp: int = 0
       self.speed: int = 0
       self.error_times: int = 0
       self.is_frame_ready: bool = False
       self.is_pkg_ready: bool = False
       self.data_tmp: bytearray = bytearray()  # Buffer for incoming bytes
       self.frame_temp: List[PointData] = []     # Collected points for one full scan cycle
       self.one_pkg: List[Optional[PointData]] = [None] * 12  # One packet holds 12 points
       self.output: dict = {}                    # Final laserscan-like output


   def get_speed(self) -> float:
       return self.speed / 360.0


   def parse(self, data: bytes) -> bool:
       """Parse incoming bytes to extract a full LiDAR packet."""
       self.data_tmp.extend(data)
       if len(self.data_tmp) < PACKET_SIZE:
           return False


       # Look for the header byte (0x54)
       start = 0
       while start < len(self.data_tmp) - 1:
           if self.data_tmp[start] == HEADER_VALUE:
               # Check if the next byte is the expected length
               if self.data_tmp[start + 1] == LENGTH_VALUE:
                   break
           start += 1


       # If we skipped any bytes, count as error and trim them
       if start > 0:
           self.error_times += 1
           self.data_tmp = self.data_tmp[start:]


       if len(self.data_tmp) < PACKET_SIZE:
           return False


       # Extract one full packet
       packet = self.data_tmp[:PACKET_SIZE]


       # Compute CRC over first 46 bytes
       crc = 0
       for b in packet[:-1]:
           crc = CRC_TABLE[(crc ^ b) & 0xff]


       # Unpack packet according to format:
       # Format: <BBHH + ("HB" * 12) + HHB
       fmt = "<BBHH" + "HB"*12 + "HHB"
       try:
           unpacked = struct.unpack_from(fmt, packet, 0)
       except struct.error as e:
           print("Unpack error:", e)
           self.data_tmp = self.data_tmp[1:]
           return False


       # Unpacked fields:
       # 0: header, 1: length, 2: speed, 3: start_angle,
       # 4..27: 12 pairs of (distance, confidence),
       # 28: end_angle, 29: time          stamp, 30: received CRC
       recv_crc = unpacked[30]
       if crc != recv_crc:
           self.error_times += 1
           # Remove header and try again
           self.data_tmp = self.data_tmp[1:]
           return False


       # Header and length are verified; extract fields
       self.speed = unpacked[2]
       start_angle = unpacked[3]
       end_angle = unpacked[28]
       self.timestamp = unpacked[29]


       # Compute angular difference
       diff = ((end_angle / 100.0) - (start_angle / 100.0) + 360) % 360
       # Optionally, add a sanity check for diff based on speed if desired


       # Calculate angular step for 12 points
       step = diff / (12 - 1)  # 11 intervals
       start_angle_deg = start_angle / 100.0
       end_angle_deg = (end_angle % 36000) / 100.0


       # Extract 12 points from data
       for i in range(12):
           # Each point: unpacked[4 + i*2] is distance, unpacked[4 + i*2 + 1] is confidence
           distance = unpacked[4 + i*2]
           confidence = unpacked[4 + i*2 + 1]
           angle = start_angle_deg + i * step
           if angle >= 360.0:
               angle -= 360.0
           pt = PointData(angle=angle, distance=distance, confidence=confidence)
           self.one_pkg[i] = pt
           self.frame_temp.append(pt)
       # Ensure the last point exactly matches the reported end angle
       self.one_pkg[-1].angle = end_angle_deg
       self.is_pkg_ready = True


       # Remove the processed packet from the buffer
       self.data_tmp = self.data_tmp[PACKET_SIZE:]
       return True


   def transform(self, data: List[PointData]) -> List[PointData]:
       # Optional transformation (e.g., coordinate conversion) can be applied here
       return data


   def assemble_packet(self) -> bool:
       """
       Assemble a full 360° scan from collected points.
       This version waits until the points in frame_temp cover at least 350°.
       It does so by sorting the points by angle and finding the largest gap.
       If (360 - largest_gap) is large enough, we consider the scan complete.
       """
       if not self.frame_temp:
           return False


       # Work on a copy of the points
       pts = self.frame_temp[:]
      
       # Require a minimum number of points to reduce noise (adjust threshold as needed)
       if len(pts) < 10:
           return False


       # Sort points by angle (assumed in degrees)
       pts_sorted = sorted(pts, key=lambda p: p.angle)


       # Find the largest gap between consecutive points
       max_gap = 0.0
       gap_index = 0
       for i in range(1, len(pts_sorted)):
           gap = pts_sorted[i].angle - pts_sorted[i-1].angle
           if gap > max_gap:
               max_gap = gap
               gap_index = i
       # Also consider the wrap-around gap (between the last and first point)
       wrap_gap = pts_sorted[0].angle + 360 - pts_sorted[-1].angle
       if wrap_gap > max_gap:
           max_gap = wrap_gap
           gap_index = 0


       # The effective angular coverage is 360 minus the largest gap.
       coverage = 360 - max_gap
       # If the coverage is less than, say, 350 degrees, wait for more points.
       if coverage < 350:
           return False


       # Reassemble the full revolution:
       # Remove the gap by "cutting" the circle at the largest gap index.
       full_scan = pts_sorted[gap_index:] + pts_sorted[:gap_index]


       # Clear the frame_temp as these points have been processed.
       self.frame_temp = []


       # Optionally, transform the scan (e.g., for coordinate adjustments)
       full_scan = self.transform(full_scan)


       # Convert the full revolution into a laserscan structure.
       self.to_laserscan(full_scan)
       self.is_frame_ready = True
       return True


   def to_laserscan(self, src: List[PointData]):
       """
       Convert the assembled full revolution (src) into a laserscan structure
       that covers 0 to 360° regardless of the angles in src.
       """
       # Force a full 360° scan.
       angle_min = 0.0
       angle_max = 2 * math.pi
       full_resolution = 360  # e.g., one beam per degree (adjust as desired)
       angle_increment = (angle_max - angle_min) / full_resolution


       self.output = {
           "header": {"frame_id": "base_scan", "stamp": self.timestamp},
           "angle_min": angle_min,
           "angle_max": angle_max,
           "range_min": 0.0,
           "range_max": 100.0,
           "angle_increment": angle_increment,
           "time_increment": 0.0,  # can be set as needed
           "scan_time": 360 / self.speed if self.speed else 0.0,
           "ranges": [float('nan')] * full_resolution,
           "intensities": [float('nan')] * full_resolution,
       }


       # For each point in the full revolution, normalize its angle to [0,360) degrees,
       # then compute its corresponding index in the full scan.
       for pt in src:
           # Normalize the angle to [0, 360)
           norm_angle = pt.angle % 360
           angle_rad = math.radians(norm_angle)
           index = int(angle_rad / angle_increment) % full_resolution
           range_val = pt.distance / 1000.0  # Convert mm to meters if needed
           # If multiple points fall into the same bin, choose the closer one.
           if math.isnan(self.output["ranges"][index]) or self.output["ranges"][index] is None or range_val < self.output["ranges"][index]:
               self.output["ranges"][index] = range_val
               self.output["intensities"][index] = pt.confidence
           """               
       # Filtering operation
       distances = np.array(self.output["ranges"])
       # Remove NaN and 0.0 values
       valid_data = distances[~np.isnan(distances)]  # Remove NaN
       valid_data = valid_data[valid_data > 0]  # Remove 0.0 values


       # Interpolating missing values (optional)
       df = pd.DataFrame(distances, columns=['Distance'])
       df['Interpolated'] = df['Distance'].replace(0, np.nan).interpolate()


       # Update the ranges with interpolated values
       self.output["ranges"] = df['Interpolated'].tolist()"""


   def get_pkg_data(self) -> List[Optional[PointData]]:
       self.is_pkg_ready = False
       return self.one_pkg


# --------------------------------------------------------------------
# Asyncio Protocol for Serial Port Handling
# --------------------------------------------------------------------
class SerialProtocol(asyncio.Protocol):
   def __init__(self, lidar: LiPkg, callback):
       self.lidar = lidar
       self.callback = callback


   def connection_made(self, transport):
       self.transport = transport
       print("Serial port opened:", transport)


   def data_received(self, data: bytes):
       # Feed incoming bytes to the LiPkg parser.
       if self.lidar.parse(data):
           if self.lidar.assemble_packet():
               # Execute callback when a full scan cycle is assembled.
               self.callback(self.lidar.output)


   def connection_lost(self, exc):
       print("Serial port closed")
       asyncio.get_event_loop().stop()


# ---------------------------
# Asyncio Serial Reader (Background Thread)
# ---------------------------
async def main_async(port: str, baudrate: int):
   lidar = LiPkg()


   # 실시간 플로팅 설정
   plt.ion()
   fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}) 
   times = 5
   def cycle_callback(scan):
       # When a full revolution is assembled, print the distance data.
       #distances = scan["ranges"]
        distance_history = np.zeros((times, len(scan["ranges"])))

        for i in range(times):  distance_history[i] = scan["ranges"]

        avg_distance = np.nanmean(np.where(distance_history == 0, np.nan, distance_history), axis=0)    
        angles = np.linspace(0, 360, len(avg_distance))
        angles = np.where(angles > 180, angles - 360, angles)

        #print("Distances:", avg_distance)

        # 최소 거리 5개 찾기
        min_indices = np.argpartition(avg_distance, 5)[:5]
        min_distances = avg_distance[min_indices]
        min_angles = angles[min_indices]

        # 노이즈 제거 (이상값 필터링)
        mean_dist = np.mean(min_distances)
        std_dist = np.std(min_distances)
        valid_mask = (min_distances > mean_dist - 2 * std_dist) & (min_distances < mean_dist + 2 * std_dist)

        # 필터링된 거리 및 각도
        filtered_distances = np.mean(min_distances[valid_mask])
        filtered_angles = np.mean(min_angles[valid_mask])
        #print(f"Filtered distances: {filtered_distances}m at {filtered_angles}°")


        # find shortest distance and corresponding angle
        #min_distance = np.nanmin(avg_distance)
        #min_angle = angles[np.nanargmin(avg_distance)]
        distance_threshold = 1.5
        if(filtered_distances < distance_threshold):
            print(f"Shortest distance: {filtered_distances:.3f} m at {filtered_angles:.3f}°")
        else:  
            print("No obstacle detected")

        """
        # 기존 플롯 지우고 새 데이터로 업데이트
        ax.clear()
        ax.set_ylim(0, 2)  # Ensure the y-axis limit is set to 20 cm
        ax.set_title("LIDAR Distance Data")
        ax.plot(np.radians(angles), avg_distance, 'b.')

        plt.draw()
        plt.pause(1e-10) # Update plot every 0.1 seconds
        """

   loop = asyncio.get_running_loop()
   transport, protocol = await serial_asyncio.create_serial_connection(
       loop,
       lambda: SerialProtocol(lidar, cycle_callback),
       port,
       baudrate=baudrate
   )
  
   try:
       await asyncio.Future()  # Run indefinitely.
   except KeyboardInterrupt:
       transport.close()


def start_async_loop(port: str, baudrate: int):
   asyncio.run(main_async(port, baudrate))


# ---------------------------
# Main Entry Point
# ---------------------------
if __name__ == "__main__":
   import sys
   port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
   baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
   start_async_loop(port, baudrate)
  








