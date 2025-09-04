# PennAir Shape Detection (3D Calibration)

This repository contains Python scripts for detecting shapes in images and videos, and estimating their 3D positions relative to the camera using OpenCV. The depth calibration is performed using a reference circle of known real-world size (10 inches radius).

## Requirements
- Python 3.7+
- OpenCV (`opencv-python`)
- NumPy

Install dependencies:
```bash
pip install opencv-python numpy
```

## Files
- `3d_detection.py`: Detects shapes in a video and estimates their 3D coordinates using the reference circle for depth calibration.
- `static_shape_detection.py`: Detects shapes in a static image and marks their centers.
- `video_shape_detection.py`: Detects shapes in a video and marks their centers (2D only).

## How to Run

### 3D Shape Detection (Video)
This script uses the reference circle to calibrate depth and applies it to all detected shapes.

```bash
python 3d_detection.py
```

- By default, it processes `PennAir 2024 App Dynamic Hard.mp4`. You can change the video file by editing the `file_path` variable in the script.
- The script will display several OpenCV windows showing the detection results. Press `q` to quit.

### Static Image Shape Detection
```bash
python static_shape_detection.py
```
- By default, it processes `PennAir 2024 App Static(1).png`. You can change the image file by editing the script.

### Video Shape Detection (2D)
```bash
python video_shape_detection.py
```
- By default, it processes `PennAir 2024 App Dynamic Hard.mp4`.


## Background Agnosticism

To achieve background agnosticism, the shape detection algorithms use a combination of adaptive thresholding, edge detection, and morphological operations. These steps help to:
- Suppress background textures and lighting variations.
- Enhance the edges of foreground shapes regardless of the background.
- Remove small noise and connect broken edges, making the detection robust to different backgrounds.

By focusing on contours and their geometric properties (such as area, perimeter, and circularity), the algorithms can reliably identify and process shapes even when the background changes or contains clutter.

## Algorithm Performance and Adjustments

### Performance
- The algorithm performs well in detecting and marking shapes in both static images and video frames, with varying backgrounds and shape colors.

### Adjustments for Efficiency
- Morphological operations (close/open) are used with moderate kernel sizes and limited iterations to balance noise reduction and computational cost.
- Only large contours are processed, skipping small/noisy regions to further reduce computation.
