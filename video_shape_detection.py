import cv2
import numpy as np

file_path = "resources/PennAir 2024 App Dynamic Hard.mp4" #can replace with dynamic or dynamic hard
cap = cv2.VideoCapture(file_path)

def detect_shapes(img):
    img_copy = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    #get edges
    edges = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                  cv2.THRESH_BINARY_INV, 11, 2)

    #dilate edges to connect broken lines
    kernel = np.ones((3, 3), np.uint8)
    dilated_edges = cv2.dilate(edges, kernel, iterations=2)
    inverted = cv2.bitwise_not(dilated_edges)

    #morphology to clean noise
    kernel = np.ones((3,3), np.uint8)
    cleaned = cv2.morphologyEx(inverted, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Filter out small shapes caused by remaining noise
    large_contours = []
    for cnt in contours:
        perimeter = cv2.arcLength(cnt, True)
        area = cv2.contourArea(cnt)
        if area > 5000 and (perimeter / area) < 0.15:
            large_contours.append(cnt)
    cv2.drawContours(img, large_contours, -1, (255, 255, 255), 2)

    #get shape centers
    for cnt in large_contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
            cv2.putText(img, f"Center ({cX}, {cY})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cX, cY = 0, 0

    # Display the result
    cv2.imshow('Threshold', edges)
    cv2.imshow('Cleaned', cleaned)
    cv2.imshow('Dilated Edges', dilated_edges)
    cv2.imshow('Detected Shapes', img)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Call the shape detection function
    detect_shapes(frame)

    # Wait for a key press: 'q' to quit, otherwise continue
    key = cv2.waitKey(30) & 0xFF  # 30ms delay for ~33fps
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()