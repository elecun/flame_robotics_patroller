import velodyne_decoder as vd

pcap_file = '21-3.pcap'
cloud_arrays = []
for stamp, points in vd.read_pcap(pcap_file):
    cloud_arrays.append(points)

import time
import numpy as np
import open3d as o3d

for i, points in enumerate(cloud_arrays):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(np.tile(points[:, 3:4] / 255.0, (1, 3)))

    print(f"Showing frame {i}/{len(cloud_arrays)}")
    o3d.visualization.draw_geometries([pcd])
    time.sleep(0.1)