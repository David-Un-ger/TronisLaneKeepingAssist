# C++ Lane Keeping Assistant for 3D Unreal Engine Simulator

This repository provides 3 classes (lane detection, velocity control, steering control) for a lane keeping assistant in C++. 
The classes were created using the Unreal Engine based 3D simulator Tronis.

Input for the algorithm:
- Camera imaga of front mono camera
- current speed
- object list to determine distance from vehile driving ahead

## Part 1: Lane detection
This part of the readme discussed the class LaneFinder, which inputs an RGB image and outputs a point towards which the car should steer to keep the lane. 

The implementation is based on classical algorithms and uses OpenCV. Lane detection using deep learning would make the algorithm more flexible, but is not part of this repo.

Hint: If you want to implement this on your own and from scratch, two things will speed up the implementation process:
1. Record a video while driving manually and then load the recorded video while implementing the lane detection
2. Use sliders to adjust your parameters (hence you will need to define several thresholds for the implementation, sliders are a convenient way to adapt the parameters on the fly)

The following figure shows several steps of the implementation:
![2022-04-11-10-06-34](https://user-images.githubusercontent.com/35065831/163670949-6a5843b4-3a70-45a8-9e99-46732bafd6a6.gif)

### Step 1: Lane marking detection
As the first step the lane markings must be detected. It is clear that the lane markings will be on the street, why only a part of the image is relevant. A crop of the image gives the relevant part.

``` 
Rect roi(0, 260, 720, 100);  // x,y, width, height
Mat imgCut = imgOrig(roi); 
```

Cropping the image results in a much smaller image and less processing effort.
<img src="https://user-images.githubusercontent.com/35065831/162695739-7749e681-de9d-47b4-a3c6-d1cc6a4d9cb9.jpg" alt="crop" width="400"/>


Two properties can be used to find the relevant pixels that belong to a lane marking: the color (hence lane markings are white) and the gradient (hence the markings are of different color than the street).

To find and select only "white" pixels, that are likely to belong to a line, the image is first converted into HSV space (HSV is much more discriminative between the different colors) and then the `inRange()` function is used.

```
cvtColor(imgCut, imgHSV, COLOR_BGR2HSV);
int huemin = 73, satmin = 0, valmin = 96;
int huemax = 140, satmax = 175, valmax = 255;
Scalar lower(huemin, satmin, valmin);
Scalar upper(huemax, satmax, valmax);
inRange(imgHSV, lower, upper, imgColorThresh);
```

To find lanes assuming they have a high gradient, 

```
cvtColor(imgCut, imgGray, COLOR_BGR2GRAY);
GaussianBlur(imgGray, imgBlur, Size(3, 3), 3, 3);
Canny(imgBlur, imgCanny, cannyLow, cannyHigh);
```

### Step 2: Bird´s Eye View Transformation

### Step 3: Contour Finding

### Step 4: Polynom fitting

### Step 5: Steering target finding

<img src="https://user-images.githubusercontent.com/35065831/162695792-9a568e40-dee8-4cde-bbd9-d3ecfd7f794e.jpg" alt="input image" width="400"/>


Detect the lane in which a car is driving and control the acceleration and steering that the car keeps its lane




Final result
![final](https://user-images.githubusercontent.com/35065831/162695892-27b9ad8b-7c93-46de-8e27-83727465d0da.jpg)
