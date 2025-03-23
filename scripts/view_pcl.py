import sys

import open3d as o3d 


def show_pcl(filepath: str) -> None:
    cloud = o3d.io.read_point_cloud(filepath)
    o3d.visualization.draw_geometries([cloud])  

if __name__ == "__main__":
    filepath = sys.argv[1]
    show_pcl(filepath)
