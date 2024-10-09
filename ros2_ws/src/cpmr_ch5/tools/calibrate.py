import glob
import json
import cv2
import numpy as np

#
# A wrapper for the opencv calibration package. The following must correspond to your calibration
# target
#
n_col = 9
n_row = 6
square_size = 0.025 
  
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 

# the calibration points in a space that matches the target
object_points_3D = np.zeros((n_row * n_col, 3), np.float32)  
object_points_3D[:,:2] = np.mgrid[0:n_row, 0:n_col].T.reshape(-1, 2) 
 
object_points_3D = object_points_3D * square_size
 
object_points = []
image_points = []
  
      
images = glob.glob('frame_*.jpg')
print(images)
for image_file in images:
    print(f'Processing calibration image {image_file}')
   
    image = cv2.imread(image_file)  
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
    success, corners = cv2.findChessboardCorners(gray, (n_col, n_row), None)
    if success == True:
        object_points.append(object_points_3D)
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)       
        image_points.append(corners)
        cv2.drawChessboardCorners(image, (n_row, n_col), corners, success)
        cv2.imshow("calibration points", image) 
        cv2.waitKey(1 * 1000)  # show for 1 second
    else:
        print('unable to find calibration points for image')

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)
calib = {'K': mtx.tolist(), 'D': dist.tolist()}
print(calib)
with open ('calib.json', 'w') as f:
    json.dump(calib, f)

