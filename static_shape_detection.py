import cv2
import numpy as np

def detect_shapes(image_path):
    img = cv2.imread(image_path)
    img_copy = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #get edges
    edges = cv2.Canny(gray, 50, 150)

    #dilate edges to connect broken lines
    kernel = np.ones((2, 2), np.uint8)
    dilated_edges = cv2.dilate(edges, kernel, iterations=1)
    inverted = cv2.bitwise_not(dilated_edges)

    #morphology to clean noise
    kernel = np.ones((3,3), np.uint8)
    cleaned = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Filter out small shapes caused by remaining noise
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]
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

    # Save the detected shapes image for download
    output_path = "detected_shapes_output.png"
    cv2.imwrite(output_path, img)
    print(f"Detected shapes image saved as {output_path}")
    # Display the result
    cv2.imshow('Original Image', img_copy)
    cv2.imshow('Dilated Edges', dilated_edges)
    cv2.imshow('Detected Shapes', img)

detect_shapes("resources/PennAir 2024 App Static(1).png")

cv2.waitKey(0)
cv2.destroyAllWindows()