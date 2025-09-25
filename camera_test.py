import cv2

index = 1
backends = [
    (cv2.CAP_DSHOW, 'dshow'),
    (cv2.CAP_MSMF, 'msmf'),
    (cv2.CAP_VFW, 'vfw'),
]

for backend, backend_name in backends:
    print(f"Trying camera index {index} with backend {backend_name}...")
    cap = cv2.VideoCapture(index, backend)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            cv2.imshow(f"Camera {index} ({backend_name})", frame)
            filename = f"camera_test_{index}_{backend_name}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved image as {filename}")
            cv2.waitKey(2000)
            cv2.destroyAllWindows()
        else:
            print(f"Camera index {index} opened with {backend_name} but failed to capture image.")
        cap.release()
    else:
        print(f"Camera index {index} not available with backend {backend_name}.") 