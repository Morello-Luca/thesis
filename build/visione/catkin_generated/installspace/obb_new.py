import rospy
import numpy as np
import ros_numpy 
import pandas as pd
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped
import open3d as o3d
import matplotlib.pyplot as plt


cal = np.genfromtxt('/home/franka/test_dual_arm/src/visione/src/camera_calib_debug_dual.csv')

centroid = np.zeros(3)
dimension = np.zeros(3)

def contactPoints(obb, Rot):
    global centroid
    global dimension
    # 6.1 CENTROID CALCULATION
    centroid = np.array(obb.get_center())
    dimension = np.array(obb.extent)
    print("dimension", dimension)
    print("centroid", centroid)
    centroid_LR = centroid + np.array([[-0.5, 0, 0], [0.5, 0, 0]]) * dimension[[0]]  # x
    centroid_UD = centroid + np.array([[0, -0.5, 0], [0, 0.5, 0]]) * dimension[[1]]  # y
    centroid_FB = centroid + np.array([[0, 0, -0.5], [0, 0, 0.5]]) * dimension[[2]]  # z
    # 6.2 INSERT POINT CONTACT INTO POINT CLOUD
    pcPoints = o3d.geometry.PointCloud()
    pcPoints.points = o3d.utility.Vector3dVector(np.vstack((centroid_LR, centroid_UD, centroid_FB)))
    pcPoints.rotate(Rot)
    return pcPoints


class PointCloudProcessor:
    def __init__(self):
        self.point_cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud_cb)
        self.vector_pub = rospy.Publisher('vector_topic', Vector3Stamped, queue_size=10)

    def point_cloud_cb(self, msg):
        # Convert PointCloud2 to numpy array
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        
        points_trans = np.transpose(points)
        ones = np.ones((1, points_trans.shape[1]))
        points_trans = np.concatenate((points_trans, ones), axis=0)
        global cal
        points_cal = cal@points_trans
        points = np.transpose(points_cal[:-1])
        

        threshold_x = [0.1, 0.7]      # sx dx
        threshold_y = [-1.2, -0.2]    # h_l h_h
        threshold_z = [0.1, 0.5]      # dietro davanti
        
        df = pd.DataFrame(points, columns=['x', 'y', 'z'])
    # Applica i threshold lungo le dimensioni x, y e z
        df = df[(df['x'] >= threshold_x[0]) & (df['x'] <= threshold_x[1])]
        df = df[(df['y'] >= threshold_y[0]) & (df['y'] <= threshold_y[1])]
        df = df[(df['z'] >= threshold_z[0]) & (df['z'] <= threshold_z[1])]

        points_cropped = df.values 

        points_vis = o3d.geometry.PointCloud()
        points_vis.points = o3d.utility.Vector3dVector(points_cropped)


        voxel_size = 0.015                             # PARAM TO SETUP
        pcd_downsampled = points_vis.voxel_down_sample(voxel_size=voxel_size)

        # PRINT CLOUD POINTS PRE-POST DOWNSAMPLING
        # print("pcd",points_vis)                            # PRE  campionamento
        # print("pcd_downsampled",pcd_downsampled)              # POST campionamento


        # print("Statistical oulier removal")
        filtered_pcd = pcd_downsampled.remove_statistical_outlier(nb_neighbors=200,std_ratio=0.3)
        outliers = pcd_downsampled.select_by_index(filtered_pcd[1],invert=True)
        outliers.paint_uniform_color([1,0,0])
        filtered_pcd=filtered_pcd[0]
        #o3d.visualization.draw_geometries([filtered_pcd, outliers])

        print("Radius oulier removal")
        filtered_pcd_round = filtered_pcd.remove_radius_outlier(nb_points=20, radius=0.04)
        outliers = filtered_pcd.select_by_index(filtered_pcd_round[1],invert=True)
        outliers.paint_uniform_color([1,0,0])
        filtered_pcd_round =filtered_pcd_round[0]
        #o3d.visualization.draw_geometries([filtered_pcd_round, outliers])





        labels = np.array(filtered_pcd_round.cluster_dbscan(eps=0.065, min_points=40))
        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        clusters = []
        for i in range(max_label + 1):
            cluster = o3d.geometry.PointCloud()
            cluster.points = o3d.utility.Vector3dVector(np.asarray(filtered_pcd_round.points)[labels == i])
            clusters.append(cluster)
            colors = plt.get_cmap("tab10")(labels/(max_label if max_label > 0 else 1))
            colors[labels<0]=0
        for i, cluster in enumerate(clusters):
            cluster.colors = o3d.utility.Vector3dVector(colors[labels == i, :3])
        

        obbs=[]
        pcs=[]
        for i in range(max_label + 1):
            obb = clusters[i].get_minimal_oriented_bounding_box()
            obb.color = (0, 1, 0)
            obbs.append(obb)
            pc = contactPoints(obb,obb.R)
            pc.paint_uniform_color([0.0,1.0,0.0])
            pcs.append(pc)

       

            #o3d.visualization.draw_geometries([filtered_pcd]+obbs)



















        # Create a Pandas DataFrame from the points
        df = pd.DataFrame(points, columns=['x', 'y', 'z'])

        # Extract a random point from the DataFrame
 

        # Create a Vector3Stamped message
        vector_msg = Vector3Stamped()
        vector_msg.header.stamp = rospy.Time.now()
        vector_msg.vector.x = centroid[0]
        vector_msg.vector.y = centroid[1]
        vector_msg.vector.z = centroid[2]

        # Publish the Vector3Stamped message
        self.vector_pub.publish(vector_msg)

if __name__ == '__main__':
    rospy.init_node('point_cloud_processor')
    processor = PointCloudProcessor()
    rospy.spin()
