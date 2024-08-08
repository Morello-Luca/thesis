# 1. ENVIROMENT SETUP
import open3d as o3d
import numpy as np


import pandas as pd
import matplotlib.pyplot as plt




# 2. POINT CLOUD DATA PREPARATIONS

    # 2.1 LOAD PCD
pcd = o3d.io.read_point_cloud("out.ply")

cal = np.genfromtxt('/home/luca/catkin_ws/src/visione/src/camera_calib_debug_dual.csv')
cal_inv = np.linalg.inv(cal)

print("cal\n",cal)
print("cal_inv\n",cal_inv)




#print("tras",cal.iloc[:, -1].to_numpy())
colors = np.asarray(pcd.colors)
o3d.visualization.draw_geometries([pcd])
    # 2.2 CROP PCD
threshold_x = [0.2, 1.0]      # sx dx
threshold_y = [-0.7, 10.0]    # h_l h_h
threshold_z = [-1.6, -1.0]      # dietro davanti

df = pd.DataFrame(np.asarray(pcd.points), columns=['x', 'y', 'z'])
    # Applica i threshold lungo le dimensioni x, y e z
df = df[(df['x'] >= threshold_x[0]) & (df['x'] <= threshold_x[1])]
df = df[(df['y'] >= threshold_y[0]) & (df['y'] <= threshold_y[1])]
df = df[(df['z'] >= threshold_z[0]) & (df['z'] <= threshold_z[1])]
print("pre rot")
points = o3d.geometry.PointCloud()
points.points = o3d.utility.Vector3dVector(df.values)
points.colors = o3d.utility.Vector3dVector(colors[df.index.values])
o3d.visualization.draw_geometries([points])


outlier_cloud = points

    # 2.3 PCD ROTATION
# rotation_matrix = np.array([[1,0.01,0.01],[-0.02,0.84,0.54],[0,-0.54,0.84]])
# points.rotate(rotation_matrix)
# o3d.visualization.draw_geometries([points])








# 4 RANSAC PLANAR SEGMENTATION
# pt_to_plane_dist = 0.001   # SETUP TO THE WORKSPACE BASE
# plane_model, inliers = points.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
# [a,b,c,d]=plane_model
# inlier_cloud = points.select_by_index(inliers)
# planeseg = pd.DataFrame(np.asarray(inlier_cloud.points), columns=['x', 'y', 'z'])
# rest = pd.DataFrame(np.asarray(points.points), columns=['x', 'y', 'z'])
# rest = rest[(rest['y'] >= max(planeseg['y']))]
# #print(max(planeseg['y']))
# outlier_cloud = o3d.geometry.PointCloud()
# outlier_cloud.points = o3d.utility.Vector3dVector(rest.values)
# inlier_cloud.paint_uniform_color([1.0,0,0])
# outlier_cloud.paint_uniform_color([0.0,1.0,0.0])
# o3d.visualization.draw_geometries([outlier_cloud,inlier_cloud])


# def display_inlier_outlier(cloud, ind):
#     inlier_cloud = cloud.select_by_index(ind)
#     outlier_cloud = cloud.select_by_index(ind, invert=True)

#     print("Showing outliers (red) and inliers (gray): ")
#     outlier_cloud.paint_uniform_color([1, 0, 0])
#     inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
#     o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])



# 3.2 VOXEL DOWNSAMPLING
voxel_size = 0.015                             # PARAM TO SETUP
pcd_downsampled = outlier_cloud.voxel_down_sample(voxel_size=voxel_size)

# PRINT CLOUD POINTS PRE-POST DOWNSAMPLING
print("pcd",outlier_cloud)                            # PRE  campionamento
print("pcd_downsampled",pcd_downsampled)              # POST campionamento
o3d.visualization.draw_geometries([pcd_downsampled])

print("Statistical oulier removal")
filtered_pcd = pcd_downsampled.remove_statistical_outlier(nb_neighbors=160,std_ratio=0.4)
outliers = pcd_downsampled.select_by_index(filtered_pcd[1],invert=True)
outliers.paint_uniform_color([1,0,0])
filtered_pcd=filtered_pcd[0]
o3d.visualization.draw_geometries([filtered_pcd, outliers])

filtered_pcd_round = filtered_pcd

# print("Radius oulier removal")
# filtered_pcd_round = filtered_pcd.remove_radius_outlier(nb_points=55, radius=0.04)
# outliers = filtered_pcd.select_by_index(filtered_pcd_round[1],invert=True)
# outliers.paint_uniform_color([1,0,0])
# filtered_pcd_round =filtered_pcd_round[0]
# o3d.visualization.draw_geometries([filtered_pcd_round, outliers])



# 5. DBSCAN
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
o3d.visualization.draw_geometries([filtered_pcd_round]+clusters)

centroid = np.zeros(3)
dimension = np.zeros(3)
# 6 PC FUNCTION
def contactPoints(obb, Rot):
    global centroid
    global dimension
    # 6.1 CENTROID CALCULATION
    centroid[:3] = np.array(obb.get_center())
    dimension[:3] = np.array(obb.extent)
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

print("centroid_out_pre function", centroid)

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

centroid = np.append(centroid,1)
centroid = cal@ centroid[:, np.newaxis]


print("centroid_out_post function \n", centroid)


# 8. VISUALIZE RESULTS
o3d.visualization.draw_geometries([filtered_pcd_round]+clusters+obbs+pcs)
o3d.visualization.draw_geometries([points]+obbs+pcs)

