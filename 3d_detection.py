import cv2
import numpy as np

file_path = "resources/PennAir 2024 App Dynamic Hard.mp4" #can replace with dynamic or dynamic hard
cap = cv2.VideoCapture(file_path)
output_path = "detected_shapes_output.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = None

def detect_shapes(img):
    img_copy = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    #get edges
    edges = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    #dilate edges to connect broken lines
    kernel = np.ones((3, 3), np.uint8)
    dilated_edges = cv2.dilate(edges, kernel, iterations=2)
    inverted = cv2.bitwise_not(dilated_edges)

    #morphology to clean noise
    kernel = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(inverted, cv2.MORPH_CLOSE, kernel, iterations=2)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel, iterations=2)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Filter out small shapes caused by remaining noise
    large_contours = []
    for cnt in contours:
        perimeter = cv2.arcLength(cnt, True)
        area = cv2.contourArea(cnt)
        if area > 5000 and (perimeter / area) < 0.15:
            large_contours.append(cnt)
    cv2.drawContours(img, large_contours, -1, (255, 255, 255), 2)

    #setting up camera matrix
    K = np.array([[2564.3186869, 0, 0], [0, 2569.70273111, 0], [0, 0, 1]])
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    real_radius = 10.0  #note that this is in inches

    #find the circle and calibrate Z
    calibrated_Z = None
    for cnt in large_contours:
        (x, y), pixel_radius = cv2.minEnclosingCircle(cnt)
        pixel_radius = float(pixel_radius)
        circularity = cv2.arcLength(cnt, True) / (2 * np.pi * pixel_radius) if pixel_radius > 0 else 0
        if 0.95 < circularity < 1.05 and pixel_radius > 0:
            calibrated_Z = fx * real_radius / pixel_radius
            break

    #apply calibrated Z to all shapes
    for cnt in large_contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if calibrated_Z is not None:
                X = (cX - cx) * calibrated_Z / fx
                Y = (cY - cy) * calibrated_Z / fy
                cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
                cv2.putText(img, f"3D Center (X={X:.1f}, Y={Y:.1f}, Z={calibrated_Z:.1f})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
                cv2.putText(img, f"Center ({cX}, {cY})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cX, cY = 0, 0

    #display the result
    cv2.imshow('Threshold', edges)
    cv2.imshow('Cleaned', cleaned)
    cv2.imshow('Dilated Edges', dilated_edges)
    cv2.imshow('Detected Shapes', img)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Initialize VideoWriter after getting frame size
    if out is None:
        height, width = frame.shape[:2]
        out = cv2.VideoWriter(output_path, fourcc, 30, (width, height))

    detect_shapes(frame)

    # Write the frame with detected shapes to output video
    out.write(frame)

    key = cv2.waitKey(30) & 0xFF
    if key == ord('q'):
        break

cap.release()
if out is not None:
    out.release()
    print(f"Detected shapes video saved as {output_path}")
cv2.destroyAllWindows()