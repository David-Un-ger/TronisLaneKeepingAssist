# C++ Lane Keeping Assistant for 3D Unreal Engine Simulator

This repository provides 3 classes (lane detection, velocity control, steering control) for a lane keeping assistant in C++. 
The classes were created using the Unreal Engine based 3D simulator Tronis. Since Tronis is not widely used, only the classes are provided and the commands for using the Tronis API are not provided.

Input for the algorithm:
- Camera imaga of front mono camera
- current speed
- object list to determine distance from vehile driving ahead

## Part 1: Lane detection
Hint: If you want to implement this on your own and from scratch, two things will speed up the implementation process:
1. Record a video while driving manually and then load the recorded video while implementing the lane detection
2. Use sliders to adjust your parameters (hence you will need to define several thresholds for the implementation, sliders are a convenient way to adapt the parameters on the fly)

The following figure shows several steps of the implementation:
![2022-04-11-10-06-34](https://user-images.githubusercontent.com/35065831/163670949-6a5843b4-3a70-45a8-9e99-46732bafd6a6.gif)

### Step 1: Line detection

### Step 2: Bird´s Eye View Transformation

### Step 3: Contour Finding

### Step 4: Polynom fitting

### Step 5: Steering target finding

<img src="https://user-images.githubusercontent.com/35065831/162695792-9a568e40-dee8-4cde-bbd9-d3ecfd7f794e.jpg" alt="input image" width="400"/>


Detect the lane in which a car is driving and control the acceleration and steering that the car keeps its lane
![crop](https://user-images.githubusercontent.com/35065831/162695739-7749e681-de9d-47b4-a3c6-d1cc6a4d9cb9.jpg)



Final result
![final](https://user-images.githubusercontent.com/35065831/162695892-27b9ad8b-7c93-46de-8e27-83727465d0da.jpg)
