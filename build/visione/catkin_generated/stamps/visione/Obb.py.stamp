#!/usr/bin/env
# 1. ENVIROMENT SETUP
import open3d as o3d
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Vector3Stamped



# 2. POINT CLOUD DATA PREPARATIONS

    # 2.1 LOAD PCD
pcd = o3d.io.read_point_cloud("/home/luca/catkin_ws/src/visione/src/out1.ply")


colors = np.asarray(pcd.colors)
    # 2.2 CROP PCD
threshold_x = [0.2, 1.0]      # sx dx
threshold_y = [-0.7, 10.0]    # h_l h_h
threshold_z = [-1.6, -1.0]      # dietro davanti

df = pd.DataFrame(np.asarray(pcd.points), columns=['x', 'y', 'z'])
    # Applica i threshold lungo le dimensioni x, y e z
df = df[(df['x'] >= threshold_x[0]) & (df['x'] <= threshold_x[1])]
df = df[(df['y'] >= threshold_y[0]) & (df['y'] <= threshold_y[1])]
df = df[(df['z'] >= threshold_z[0]) & (df['z'] <= threshold_z[1])]

points = o3d.geometry.PointCloud()
points.points = o3d.utility.Vector3dVector(df.values)
points.colors = o3d.utility.Vector3dVector(colors[df.index.values])


outlier_cloud = points

   
# 3.2 VOXEL DOWNSAMPLING
voxel_size = 0.015                             # PARAM TO SETUP
pcd_downsampled = outlier_cloud.voxel_down_sample(voxel_size=voxel_size)

# PRINT CLOUD POINTS PRE-POST DOWNSAMPLING
print("Pre Sampling",outlier_cloud)                            # PRE  campionamento
print("Post Sampling",pcd_downsampled)              # POST campionamento

filtered_pcd = pcd_downsampled.remove_statistical_outlier(nb_neighbors=160,std_ratio=0.4)
outliers = pcd_downsampled.select_by_index(filtered_pcd[1],invert=True)
outliers.paint_uniform_color([1,0,0])
filtered_pcd=filtered_pcd[0]

filtered_pcd_round = filtered_pcd


# 5. DBSCAN
labels = np.array(filtered_pcd_round.cluster_dbscan(eps=0.065, min_points=40))
max_label = labels.max()
print(f"Point cloud has {max_label + 1} clusters")
clusters = []
for i in range(max_label + 1):
    cluster = o3d.geometry.PointCloud()
    cluster.points = o3d.utility.Vector3dVector(np.asarray(filtered_pcd_round.points)[labels == i])
    clusters.append(cluster)
colors = plt.get_cmap("tab10")(labels/(max_label if max_label > 0 else 1))
colors[labels<0]=0
for i, cluster in enumerate(clusters):
    cluster.colors = o3d.utility.Vector3dVector(colors[labels == i, :3])

centroid = np.zeros(3)
dimension = np.zeros(3)
# 6 PC FUNCTION
def contactPoints(obb, Rot):
    global centroid
    global dimension
    # 6.1 CENTROID CALCULATION
    centroid[:3] = np.array(obb.get_center())
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

# 7. FOR EACH CLUSTER DETERMINE BOUNDING BOX AND POINT CONTACTS
obbs=[]
pcs=[]
for i in range(max_label + 1):
    obb = clusters[i].get_minimal_oriented_bounding_box()


    obb.color = (0, 1, 0)
    obbs.append(obb)
    pc = contactPoints(obb,obb.R)
    pc.paint_uniform_color([0.0,1.0,0.0])
    pcs.append(pc)

# 8. VISUALIZE RESULTS

def talker():
    pub = rospy.Publisher('centroid_obj', Vector3Stamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        vector_msg = Vector3Stamped()
        vector_msg.header.stamp = rospy.Time.now()
        vector_msg.vector.x = centroid[0]
        vector_msg.vector.y = centroid[1]
        vector_msg.vector.z = centroid[2]
        pub.publish(vector_msg)
        rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('point_cloud_subscriber')
    rospy.Subscriber('point_cloud_topic', PointCloud2, callback)
    rospy.spin()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
