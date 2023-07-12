import cv2
import numpy as np
import pickle
import glob

def cam_calibrate(images, cam_calib):

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    pts = np.zeros((6 * 9, 3), np.float32)
    pts[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # capture calibration frames
    obj_points = []  # 3d point in real world space
    img_points = []  # 2d points in image plane.
    frames = []
    for fname in images:
        frame = cv2.imread(fname)
        ret = frame is not None

        if not ret:
            print("Calibrating camera...")
            cv2.destroyAllWindows()
            break

        frame_copy = frame.copy()

        corners = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retc, corners = cv2.findChessboardCorners(gray, (9, 6), None)
        if retc:
            print("found points")
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            # Draw and display the corners
            cv2.drawChessboardCorners(frame_copy, (9, 6), corners, ret)

            cv2.imshow('points', frame_copy)
            # s to save, c to continue, q to quit
            if cv2.waitKey(0) & 0xFF == ord('s'):
                img_points.append(corners)
                obj_points.append(pts)
                frames.append(frame)
                cv2.imwrite('./results/' + fname.split('_')[1], frame_copy)
            elif cv2.waitKey(0) & 0xFF == ord('c'):
                continue
            elif cv2.waitKey(0) & 0xFF == ord('q'):
                print("Calibrating camera...")
                cv2.destroyAllWindows()
                break
        else:
            cv2.imwrite('./results/' + fname.split('_')[1], frame_copy)

    # compute calibration matrices

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, frames[0].shape[0:2], None, None)

    # check
    error = 0.0
    for i in range(len(frames)):
        proj_imgpoints, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        error += (cv2.norm(img_points[i], proj_imgpoints, cv2.NORM_L2) / len(proj_imgpoints))
    print("Camera calibrated successfully, total re-projection error: %f" % (error / len(frames)))

    cam_calib['mtx'] = mtx
    cam_calib['dist'] = dist
    print("Camera parameters:")
    print(cam_calib)

    pickle.dump(cam_calib, open("calib_cam.pkl", "wb"))


if __name__ == "__main__":

    cam_calib = {}

    images = glob.glob('./raw/*.jpg')

    cam_calibrate(images, cam_calib)
