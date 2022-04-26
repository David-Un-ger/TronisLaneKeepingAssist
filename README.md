# C++ Lane Keeping Assistant for 3D Unreal Engine Simulator

This repository provides 3 classes (lane detection, steering control, velocity control) for a lane keeping assistant in C++. 
The classes were created using the Unreal Engine based 3D simulator Tronis.

Input for the algorithm:
- camera image of front monocular camera
- current speed
- object list to determine distance from vehicle driving ahead

## Part 1: Lane detection
This part of the readme discusses the class `LaneFinder`, which inputs an RGB image and outputs a point towards which the car should steer to keep the lane. 

The implementation is based on classical algorithms and uses OpenCV. Lane detection using deep learning would make the algorithm more flexible, but is not part of this repo.

Hint: If you want to implement this on your own and from scratch, two things will speed up the implementation process:
1. Record a video while driving manually and then load the recorded video while implementing the lane detection
2. Use sliders to adjust your parameters (hence you will need to define several thresholds for the implementation, sliders are a convenient way to adapt the parameters on the fly)

The following figure shows several steps of the implementation that are explained in the following:
![2022-04-11-10-06-34](https://user-images.githubusercontent.com/35065831/163670949-6a5843b4-3a70-45a8-9e99-46732bafd6a6.gif)

### Step 1: Lane marking detection
As the first step the lane markings must be detected using the image of the front camera.

<img src="https://user-images.githubusercontent.com/35065831/162695792-9a568e40-dee8-4cde-bbd9-d3ecfd7f794e.jpg" alt="input image" width="400"/>

It is clear that the lane markings will be on the street, why only a part of the image is relevant. A crop of the image gives the relevant part.

``` 
Rect roi(0, 260, 720, 100);  // x, y, width, height
Mat imgCut = imgOrig(roi); 
```

Cropping the image results in a much smaller image and less processing effort.

<img src="https://user-images.githubusercontent.com/35065831/162695739-7749e681-de9d-47b4-a3c6-d1cc6a4d9cb9.jpg" alt="crop" width="400"/>


Two properties can be used to find the relevant pixels that belong to a lane marking: the color (hence lane markings are white) and the gradient (hence the markings are of different color than the street).

To find and select only "white" pixels that are likely to belong to a line, the image is first converted into HSV space (HSV is much more discriminative between the different colors) and then the `cv.inRange()` function is used.

```
cvtColor(imgCut, imgHSV, COLOR_BGR2HSV);
int huemin = 73, satmin = 0, valmin = 96;
int huemax = 140, satmax = 175, valmax = 255;
Scalar lower(huemin, satmin, valmin);
Scalar upper(huemax, satmax, valmax);
inRange(imgHSV, lower, upper, imgColorThresh);
```

To find lanes assuming they have a high gradient, the image is first converted into grayscale, then blurred and afterwards a Canny edge detector is used.

```
cvtColor(imgCut, imgGray, COLOR_BGR2GRAY);
GaussianBlur(imgGray, imgBlur, Size(3, 3), 3, 3);
Canny(imgBlur, imgCanny, cannyLow, cannyHigh);
```

Both information is combined using a logical AND operator. Thus, only image pixels that have a high gradient and are white are detected as a lane marking.

The resulting image looks like follows:

<img src="https://user-images.githubusercontent.com/35065831/163925446-84bb89ec-ebf5-4e36-a2fc-c1ca921a195d.jpg" alt="color_canny" width="400"/>


### Step 2: Bird´s Eye View Transformation

Obviously, the image lane markings are no line and are hard to detect in the perspective space. In the Bird's Eye View (BEV) the lanes are much easier to detect, which is why the image from step 1 is transformed into BEV. Each 4 input and 4 target points are required to obtain the transformation matrix M using `cv.getPerspectiveTransform()`.

Both the transformation matrix and the lane image are fed into the `cv.warpPerspective()` function which outputs the lane markings in BEV.

<img src="https://user-images.githubusercontent.com/35065831/163928386-154da8c1-2bf2-435a-bf97-862ef9d0717c.jpg" alt="bev" width="200"/>

### Step 3: Contour Finding

The function `findLineContours()` uses `cv.findContours()` to find the contours in the BEV image that correspond to the left and right lane marking. Therefore, two checks are performed:
- is the contour large enough to be a lane marking (this is useful to omit some errors in preceding steps)
- does the contour belong to the left or to the right lane marking

If no fitting lane markings are found, old lane markings from the last processing steps are used as backup. 

### Step 4: Steering target finding

The contours for each lane marking are useful for estimating the steering command directly. The function `contours2Points()` is used to output a steering target value that can be used to control the vehicle.

The contours for the left and right line each consist of a vector of points. A more useful representation for each lane marking is a second order polynomial that can be found using the function `PolynomialFit()` function. Given the two polynomials in the BEV space, a steering target value can be computed assuming the one always wants to steer to the middle of the lane markings. A x value (in front of the vehicle) corresponding to a distance of ~8m is plugged into both polynomials.
The arithmetic mean of both obtained y values yields the target value for the steering.

Just for visualization purposes, the detected contour for the left line is marked in red, the right line is marked in blue and both resulting polynomials are indicated by green dots. The steering target 8m in front of the car is shown using a white circle.

<img src="https://user-images.githubusercontent.com/35065831/163937474-c8cffa6e-304c-4218-80d0-d96ba8d5b2fb.jpg" alt="bev_lines" width="200"/>

As a final step for the `LaneFinder`, the BEV image is transformed back into the perspective image and superimposed to the input image: 
<img src="https://user-images.githubusercontent.com/35065831/162695892-27b9ad8b-7c93-46de-8e27-83727465d0da.jpg" alt="final" width="400"/>

This last step is also just for visualization and not required to steer the vehicle. 


## Part 2: Steering Control


## Part 3: Velocity Control
