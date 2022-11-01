import cv2
import numpy as np
import os
import sys
import argparse

# function to calibrate a stereo camera pair
# ldir (str): path to directory of calibration images of left camera
# rdir (str): path to directory of calibration images of right camera
# savedir (str): path to directory of saved calibration results
# sq_size (float): dimension of checkerboard square side (in metres)
# bd_size (int, int): dimensions of the checkerboard, as in (width, height) of square corners
def calibrate(ldir, rdir, savedir, sq_size, bd_size):    
    # generate real-word coordinates for square corners
    sq_points = np.zeros((1, bd_size[0] * bd_size[1], 3), np.float32)
    sq_points[0,:,:2] = (np.mgrid[0:bd_size[0], 0:bd_size[1]]*sq_size).T.reshape(-1, 2)
    # initialise lists for storing calibration data
    obj_points = []
    img_points_l = []
    img_points_r = []
    # sorted lists of calibration filenames   
    l_files = sorted(os.listdir(ldir))
    r_files = sorted(os.listdir(rdir))

    # loop through calibration images
    # NOTE: images must be in corresponding alphabetical order
    for path_l, path_r in zip(l_files, r_files):
        # read images, save image dimensions, convert to grayscale
        img_l = cv2.imread(ldir + path_l)
        img_r = cv2.imread(rdir + path_r)
        w, h = (img_l.shape[1], img_l.shape[0])
        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        # find chessboard corners
        ret_l, corners_l = cv2.findChessboardCornersSB(gray_l, bd_size, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE +\
            cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
        ret_r, corners_r = cv2.findChessboardCornersSB(gray_r, bd_size, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE +\
            cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
        # check if corners were found for both images
        if ret_l == False:
            raise ValueError("Left camera: no checkerboard corners found")
            print("Image path: " + path_l)
        if ret_r == False:
            raise ValueError("Right camera: no checkerboard corners found")
            print("Image path: " + path_r)

        # refine corner locations to subpixel accuracy
        winSize = (5, 5)
        zeroZone = (-1, -1)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
        corners_l = cv2.cornerSubPix(gray_l, corners_l, winSize, zeroZone, criteria)
        corners_r = cv2.cornerSubPix(gray_r, corners_r, winSize, zeroZone, criteria)
        # store corners' real-world coordinates and image coordinates from both images
        obj_points.append(sq_points)
        img_points_l.append(corners_l)
        img_points_r.append(corners_r)


    # calibrate both cameras separately
    ret_l, mtx_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(obj_points, img_points_l, (w,h), 
        None, None)
    ret_r, mtx_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(obj_points, img_points_r, (w,h),
        None, None)
    # perform stereo calibration with intrinsic parameters fixed
    ret_s, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F = cv2.stereoCalibrate(obj_points, img_points_l, 
        img_points_r, mtx_l, dist_l, mtx_r, dist_r, (w,h), None, cv2.CALIB_FIX_INTRINSIC)
    print("Stereo calibration error: ", ret_s)

    # rectify cameras, acquire parameters for rectified views
    Rr_l, Rr_r, P_l, P_r, Q, roi_l, roi_r = cv2.stereoRectify(mtx_l, dist_l, mtx_r, dist_r, 
        (w,h), R, T, 1, (0,0))

    # save all parameters needed for computing depth maps for rectified stereo view 
    np.savetxt(savedir + "mtx_l.txt", mtx_l)
    np.savetxt(savedir + "dist_l.txt", dist_l)
    np.savetxt(savedir + "mtx_r.txt", mtx_r)
    np.savetxt(savedir + "dist_r.txt", dist_r)
    np.savetxt(savedir + "Rr_l.txt", Rr_l)
    np.savetxt(savedir + "P_l.txt", P_l)
    np.savetxt(savedir + "Rr_r.txt", Rr_r)
    np.savetxt(savedir + "P_r.txt", P_r)
    np.savetxt(savedir + "Q.txt", Q)

    return mtx_l, dist_l, mtx_r, dist_r, Rr_l, P_l, Rr_r, P_r, Q  

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-l", "--ldir", help="left camera directory path")
    ap.add_argument("-r", "--rdir", help="right camera directory path")
    ap.add_argument("-s", "--savedir", help="calibration result save directory path")
    ap.add_argument("-sq", "--square", help="checkerboard square side dimension")
    ap.add_argument("-bd", "--board", help="checkerboard dimensions")
    args = vars(ap.parse_args())

    if not args.get("ldir", False):
        print("No path provided to left camera image directory")
        sys.exit()
    if not args.get("rdir", False):
        print("No path provided to right camera image directory")
        sys.exit()
    if not args.get("savedir", False):
        print("No path provided to calibration result save directory")
        sys.exit()
    if not args.get("square", False):
        print("No checkerboard square dimension provided")
        sys.exit()
    if not args.get("board", False):
        print("No checkerboard dimensions provided")
        sys.exit()
    
    calibrate(args.get("ldir"), args.get("rdir"), args.get("savedir"), float(args.get("square")), 
        tuple(map(int, args.get("board").split('x'))))
