import cv2
import numpy as np
import os

# === USER CONFIGURATION ===
CAMERA_ID = 0
SAVE_DIR = "calib_images"
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 0.025  # meters

# === Step 1: Capture checkerboard images ===
def capture_checkerboard_images():
    os.makedirs(SAVE_DIR, exist_ok=True)
    cap = cv2.VideoCapture(CAMERA_ID)
    count = 0

    print("Press SPACE to save image, ESC to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if found:
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, found)

        cv2.imshow('Checkerboard', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break  # ESC
        elif key == 32 and found:
            fname = os.path.join(SAVE_DIR, f"img_{count:02d}.png")
            cv2.imwrite(fname, frame)
            print(f"Saved {fname}")
            count += 1

    cap.release()
    cv2.destroyAllWindows()


# === Step 2: Calibrate camera from saved images ===
def calibrate_fisheye():
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []
    imgpoints = []
    img_shape = None

    images = [os.path.join(SAVE_DIR, f) for f in os.listdir(SAVE_DIR) if f.endswith(".png")]

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_shape is None:
            img_shape = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)
        if found:
            objpoints.append(objp)
            imgpoints.append(corners)

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rms, _, _, _, _ = cv2.fisheye.calibrate(
        objpoints, imgpoints, img_shape, K, D, None, None,
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
        cv2.fisheye.CALIB_CHECK_COND +
        cv2.fisheye.CALIB_FIX_SKEW,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    )

    print("Calibration complete. RMS error:", rms)
    np.savez("fisheye_intrinsics.npz", K=K, D=D, DIM=img_shape)
    print("Saved to fisheye_intrinsics.npz")


# === Step 3: Undistort test frame ===
def undistort_example():
    data = np.load("fisheye_intrinsics.npz")
    K = data["K"]
    D = data["D"]
    DIM = tuple(data["DIM"])

    cap = cv2.VideoCapture(CAMERA_ID)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Undistorted", undistorted)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


# === RUN THIS SCRIPT ===
if __name__ == "__main__":
    print("1. Capture images")
    capture_checkerboard_images()

    print("2. Calibrate")
    calibrate_fisheye()

    print("3. Undistort preview")
    undistort_example()
