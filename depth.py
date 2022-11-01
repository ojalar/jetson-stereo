import cv2
import numpy as np
import sys
import argparse
import matplotlib.pyplot as plt
import open3d as o3d

# example function to demonstrate generation of disparity map and 3D point cloud from a stereo pair
# paramdir (str): path to directory with loadd camera parameters
# img_path_l (str): path to image from left camera
# img_path_r (str): path to image from right camera
def stereo_depth(paramdir, img_path_l, img_path_r):
    # load images
    img_l = cv2.imread(img_path_l) 
    img_r = cv2.imread(img_path_r) 
    w, h = (img_l.shape[1], img_l.shape[0])
    gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
    
    # load parameters
    try:
        mtx_l = np.loadtxt(paramdir + "mtx_l.txt")
        dist_l = np.loadtxt(paramdir + "dist_l.txt")
        mtx_r = np.loadtxt(paramdir + "mtx_r.txt")
        dist_r = np.loadtxt(paramdir + "dist_r.txt")
        Rr_l = np.loadtxt(paramdir + "Rr_l.txt")
        P_l = np.loadtxt(paramdir + "P_l.txt")
        Rr_r = np.loadtxt(paramdir + "Rr_r.txt")
        P_r = np.loadtxt(paramdir + "P_r.txt")
        Q = np.loadtxt(paramdir + "Q.txt")
    except:
        print("Failed to load parameters, check directory path")
        raise

    # create rectification maps
    rect_map_l = cv2.initUndistortRectifyMap(mtx_l, dist_l, Rr_l, P_l, (w,h), cv2.CV_16SC2)
    rect_map_r = cv2.initUndistortRectifyMap(mtx_r, dist_r, Rr_r, P_r, (w,h), cv2.CV_16SC2)

    # rectify images
    rect_l = cv2.remap(gray_l, rect_map_l[0], rect_map_l[1], cv2.INTER_LANCZOS4,
        cv2.BORDER_CONSTANT, 0)
    rect_r = cv2.remap(gray_r, rect_map_r[0], rect_map_r[1], cv2.INTER_LANCZOS4,
        cv2.BORDER_CONSTANT, 0)

    # visualise rectified stereo pair
    plot_rect_l = cv2.resize(rect_l, (int(w/2), int(h/2)))
    plot_rect_r = cv2.resize(rect_r, (int(w/2), int(h/2)))
    plot_rect = np.concatenate((plot_rect_l, plot_rect_r), axis=1)
    cv2.imshow("rectified images", plot_rect)
    cv2.waitKey(0)
     
    # create stereo matcher, parameters can be checked from 
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html
    stereo = cv2.StereoSGBM_create(-(int(w/5/16)*16), 2*int(w/5/16)*16, 11, 12*11*11, 48*11*11,
        1, 1, 10, 200, 2, 1)
    #stereo = cv2.StereoSGBM_create(-304, 608, 11, 12*11*11, 48*11*11,
    #    1, 1, 10, 200, 2, 1)
    # generate disparity map
    disparity = stereo.compute(rect_l, rect_r)
    # display disparity map
    plt.imshow(disparity/16, 'hot')
    plt.show()
    

    # filter disparity map
    # set all negative values to the minimum value, which indicates pixels where
    # stereo matching failed
    min_d = np.amin(disparity)
    disparity[disparity <= 0 ] = min_d
    # generate point cloud with disparity map and Q-matrix (necessary stereo setup parameters)
    points3d = cv2.reprojectImageTo3D(np.float32(disparity/16), Q, handleMissingValues=True)
    # reshape point cloud to vector of (x,y,z)-values
    points = np.reshape(points3d, (-1, 3))
    
    # generate vector of 3-channel gray colors from left rectified image
    # colors correspond to points
    colors = np.reshape(rect_l/255, (-1,1))
    colors = np.repeat(colors, 3, axis=1)
    # filter out points/colors with inf-values
    colors = colors[points[:,-1] != float("inf")]
    points = points[points[:,-1] != float("inf")]
    # filter out all points where stereo matching failed (set to 10 km by opencv with flag
    # handleMissingValues=True)
    colors = colors[points[:,-1] != np.max(points[:,-1])]
    points = points[points[:,-1] != np.max(points[:,-1])]
    
    # filter out points farther away than 0.8 m to improve visualisation
    colors = colors[points[:,-1] < 0.8]
    points = points[points[:,-1] < 0.8]
    
    # visualise pointcloud with open3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    viewer = o3d.visualization.Visualizer()
    viewer.create_window()
    viewer.add_geometry(pcd)
    viewer.run()
    viewer.destroy_window()
    
    # reduce number of points for matplotlib visualisation
    # select every 75th point/color
    points = points[::75]
    colors = colors[::75]
    # create 3d matplotlib figure
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(points[:,0], points[:,2], -points[:,1], c = colors)
    ax.set_zlim(-0.1, 0.5)
    plt.show()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--pdir", help="parameter directory path")
    ap.add_argument("-l", "--imgl", help="path to left camera image")
    ap.add_argument("-r", "--imgr", help="path to right camera image")
    args = vars(ap.parse_args())

    if not args.get("pdir", False):
        print("No path provided to parameter directory")
        sys.exit()
    if not args.get("imgl", False):
        print("No path provided to left camera image")
        sys.exit()
    if not args.get("imgr", False):
        print("No path provided to right camera image")
        sys.exit()
    
    stereo_depth(args.get("pdir"), args.get("imgl"), args.get("imgr")) 
