
---
author: Lex Meulenkamp
id: 03727413 
title: BAD SLAM
subject: 3D CV and DL
---

- 1 Intro SLAM Contents
- 2 Background
   - 2.1 Direct vs Indirect
   - 2.2 Feature Correspondences
   - 2.3 Find Poses
   - 2.4 Find 3D Points
   - 2.5 Bundle Adjustment
- 3 Data Representation
   - 3.1 Surfel
   - 3.2 Keyframes
- 4 Cost function
   - 4.1 Geometric Residual
   - 4.2 Photometric Residual
- 5 Optimization
   - 5.1 Surfel Creation
   - 5.2 Surfel Normal
   - 5.3 Surfel Position and Descriptors
   - 5.4 Keyframe Pose
- 6 Benchmarks
   - 6.1 TUM RGB-D
   - 6.2 Own benchmark
      - 6.2.1 Global shutter vs rolling shutter
      - 6.2.2 Benchmark results
- 7 Last Remarks


## 1 Intro SLAM Contents

SLAM stands for simultaneously localization and mapping, where the localization refers to
the objective to know where our camera has been. In other words, determining our camera
trajectory. The mapping refers to building a 3D map, this 3D map is acquired through
inferences we do on the camera images we get from the cameras. The main idea behind
this inference process is that we can get 3D points by multiple images from the scene at
different locations. If we have completely different images from different scenes with no
correspondences it is nearly impossible to build a reliable ground-truth 3D map.
The SLAM problem is especially interesting for the robot community, if the robot knows
where it’s (localization) and how is local world is constructed (mapping), then the robot has
important information to be able to move through the physical world. Especially, if the robot
is in a location without reliable GPS information, SLAM becomes vital for an autonomous
moving robot.

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/keyframe.png)



## 2 Background

### 2.1 Direct vs Indirect

We use pixel information of images to obtain correspondences. How many pixels we use
depends on the used method. Often used terminology is direct vs indirect methods; where
direct methods use all the pixel information, and indirect only a subset of the pixels from an
image. The direct method needs an extra step in the SLAM pipeline, and that is the feature
extraction by a feature detector. Big challenge for the indirect methods is that the bundle
adjustment scheme has to optimise for more parameters due to using more pixel information.
For a long time this has been considered to be compatibility infeasible. Recently, there are
more indirect methods being published while also coming close in performance with state of
the art direct methods, and this paper is one of them which tries to tackle SLAM optimization in an indirect manner, thus skipping the feature extraction step and using all the pixels
from the images.
The correspondences between 2D images is our main ingredient to make SLAM work. Therefore, I will talk a bit more about it from the indirect method; a more studied, and for a long
time the best working method.
However, you should be aware that entire books are written just for geometric correspondences problems for computer vision, thus if you are really interested you should read some
key chapters from books like Multiple View Geometry in Computer Vision, Invitation to 3D
Vision, Introductory Techniques for 3D Computer Vision.

### 2.2 Feature Correspondences

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/features.png)

When talking about images correspondences
for the SLAM setting, Its often best to keep
it simple and get some intuition from the
case where we try to infer (pixel) correspondences between two images. How can we tell
the computer that two pixel locations both
from a different image correspond to the
same physical location from the real world?
Feature descriptors are often used to tackle
this problem within computer vision. You
might have heard of famous feature descriptors like SIFT, SURF, and ORB (which is
also used for state of the art ORB-SLAM2).
Feature descriptors find keypoints (x,y image
coordinates) within an image and each keypoint has an descriptor representation (vector).
The keypoints are in general image locations where the image gradient is high, meaning that
there is a lot of image intensity change locally. Thus, for both images we end up with a
sparse set of pixels from our image. These locations are vital information points within an
image, and you could see it as an summary of the most important information of the image.
Every keypoints has it’s own descriptor. To get the correspondences we match the descriptor
vectors by a certain distance metric between image one and two. Finding the best matches
between descriptors is an important job for the rest of the SLAM pipeline. If the matches
are bad then optimizing it with schemes such as bundle adjustment is fruitless.
Some important techniques to make better matches are used are thresholding the max distance and RANSAC.

### 2.3 Find Poses

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/pose.png)

Lets say we initialise our first camera to identity pose, thus the extrinsic matrix for the
first camera is the identity matrix. At this
stage of the pipeline we have feature correspondences between two images and the
first camera pose. Now we have enough
information to calculate our first guess for
the second camera pose. Essentially, finding
poses means finding a (extrinsic) matrix that
change a set of points to a different base in
3D space. The matrix that describes this relation between a pair of correspondences is
the essential matrix. For all our <a href="https://www.codecogs.com/eqnedit.php?latex=$x$-$x^'$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$x$-$x^'$" title="$x$-$x^'$" /></a> feature
pairs in homogeneous coordinates holds <a href="https://www.codecogs.com/eqnedit.php?latex=$x'&space;F&space;x&space;=&space;0$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$x'&space;F&space;x&space;=&space;0$" title="$x' F x = 0$" /></a>. There are different n-point algorithms;
where the n corresponds to the feature correspondences we give the algorithm which estimates the essential matrix. A well known one, is the 8 point algorithm which needs 8 feature
correspondences and gives us an estimate of the essential matrix. From the essential matrix
we can extract the second pose.
According to Harley and Zisserman [1],’For a given essential matrix <a href="https://www.codecogs.com/eqnedit.php?latex=$E&space;=&space;U&space;diag(1,1,0)&space;V^T$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$E&space;=&space;U&space;diag(1,1,0)&space;V^T$" title="$E = U diag(1,1,0) V^T$" /></a>,
and the first camera matrix <a href="https://www.codecogs.com/eqnedit.php?latex=$P&space;=&space;[I|0]$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$P&space;=&space;[I|0]$" title="$P = [I|0]$" /></a>, there are four possible choices for the second camera
matrix P’, namely
<a href="https://www.codecogs.com/eqnedit.php?latex=$P'&space;=&space;[UWV^T|&plus;u3]$\quad&space;or\quad$[UWV^T|-u3]$&space;\quad&space;or\quad&space;$[UW^TV^T|&plus;u3]$&space;\quad&space;or\quad&space;$[UW^TV^T|-u3]$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$P'&space;=&space;[UWV^T|&plus;u3]$\quad&space;or\quad$[UWV^T|-u3]$&space;\quad&space;or\quad&space;$[UW^TV^T|&plus;u3]$&space;\quad&space;or\quad&space;$[UW^TV^T|-u3]$" title="$P' = [UWV^T|+u3]$\quad or\quad$[UWV^T|-u3]$ \quad or\quad $[UW^TV^T|+u3]$ \quad or\quad $[UW^TV^T|-u3]$" /></a>
Here P is also assumed to be the identity pose just liked we did for our very first camera,
but the notation is in a homogeneous coordinate system. Here the authors refer to the SVD
composition of the matrix and u3 is the homogenous translation vector.
Thus, all we need to estimate a new camera pose in the future is 8 feature correspondences
between this camera and another camera. At first we don’t have any camera poses that’s
why we set only the first one to identity.

### 2.4 Find 3D Points

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/triangulation.png)

At this stage, we can assume that we have
two camera extrinsic matrices with their
corresponding feature correspondences. For
each feature correspondence there should be
one 3D point in the world space. This 3D
point is the physical location both camera’s
captured, but just from a different pose.
With triangulation we acquire the 3D world
point by using the 2D-2D feature correspondences. Theoretically, this should be the
physical point in the real world we measured. Thus this should be the last step of
our pipeline. However, in practice we rarely have that the assumptions in theory hold 100%.
For instance: our sensors performance might deviate resulting into noise measurements. If
we ignore this practical reality, then we accumulate small errors over time which become
globally large errors over time. The next step which is also the main part of the paper, tries
to optimise for the estimation errors we make in the front-end.

### 2.5 Bundle Adjustment

Bundle adjustment (BA) is a scheme which minimizes the reprojection loss, which can be
formulated as:
<a href="https://www.codecogs.com/eqnedit.php?latex=$L&space;=&space;\left&space;\|x_l&space;-&space;\pi(Rx_w&space;&plus;&space;t)\right&space;\|^2$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$L&space;=&space;\left&space;\|x_l&space;-&space;\pi(Rx_w&space;&plus;&space;t)\right&space;\|^2$" title="$L = \left \|x_l - \pi(Rx_w + t)\right \|^2$" /></a>
R is the rotation matrix and t the translation vector from the extrinsic camera of the local
camera.πis the projection function determined by the intrinsic matrix.
<a href="https://www.codecogs.com/eqnedit.php?latex=$Rx_w&space;&plus;&space;t$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$Rx_w&space;&plus;&space;t$" title="$Rx_w + t$" /></a> takes the 3D world point <a href="https://www.codecogs.com/eqnedit.php?latex=$x_w$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$x_w$" title="$x_w$" /></a> we used to build the map to the 3D space of the local camera.
π projects this 3D point to it’s local 2D image plane.
If our estimated camera’s poses and triangulated 3D points are good, we should get small
errors (residuals) between the projection of <a href="https://www.codecogs.com/eqnedit.php?latex=$x_w$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$x_w$" title="$x_w$" /></a> (global 3D point) for each <a href="https://www.codecogs.com/eqnedit.php?latex=$x_l$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$x_l$" title="$x_l$" /></a> (2D local
camera point) correspondence. For each 3D point in our map we look at which local features
were used to compute it, and compute the loss. BA will jointly optimise our 3D world points
(we got from triangulation), extrinsic matrices (we got from essential matrix), and optionally
intrinsic matrix if it’s not fixed. The BA scheme is a nonlinear squared error loss function;
you can choose an optimization algorithms like Gauss-Newton to find the new parameters.



## 3 Data Representation

The rest of the document is about the BAD SLAM paper [2].

### 3.1 Surfel

We have to choose a geometric primitive which we use to build our 3D map. In this paper they have chosen for surfel (3D disc) representation. The surfel does not have the
connected property like the well known polygon made by triangles (faces) and their corresponding vertices. This is beneficial in terms of processing speed for creating the 3D map
of thousands of geometric primitives. For a surfel we need to store the following attributes

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/surfel.png)

- 3D center point P
- 3D normal vector n
- Radius of the surfel r
- Scalar visual descriptor

The 3D center point and radius enables us
to know where to draw this surfel in the 3D
world. The 3D normal is mainly used for optimization purposes which we will see in the
geometric residual section 4.1. The scalar
visual descriptor is used for the photometric
error.

### 3.2 Keyframes

Lets say we have a moving camera taking pictures of a scene that we want to reconstruct. It
is not feasible for bundle adjustment to to use all the images the camera takes. Therefore,
we can only take a subset of the pictures we take. That’s why it’s called keyframes the other
frames we take are omitted. A keyframe consists of a RGB-D frame (RGB image and a grey
scale depth image) and 6 DOF (rotation and translation) camera pose. Keyframes are needed
for finding correspondences and at the same time they are needed to define a quantitative
quality of the 3D surfel for the surfels corresponding local keyframes.



## 4 Cost function
<a href="https://www.codecogs.com/eqnedit.php?latex=C(K,S)&space;=&space;\sum_{k&space;\in&space;K}\sum_{s&space;\in&space;S_k}(p_{Tukey}(\sigma^{-1}_D&space;residual_{geom}(s,k))&space;&plus;&space;w_{photo}&space;p_{Huber}(\sigma^{-1}_p&space;residual_{photo}(s,k)))" target="_blank"><img src="https://latex.codecogs.com/gif.latex?C(K,S)&space;=&space;\sum_{k&space;\in&space;K}\sum_{s&space;\in&space;S_k}(p_{Tukey}(\sigma^{-1}_D&space;residual_{geom}(s,k))&space;&plus;&space;w_{photo}&space;p_{Huber}(\sigma^{-1}_p&space;residual_{photo}(s,k)))" title="C(K,S) = \sum_{k \in K}\sum_{s \in S_k}(p_{Tukey}(\sigma^{-1}_D residual_{geom}(s,k)) + w_{photo} p_{Huber}(\sigma^{-1}_p residual_{photo}(s,k)))" /></a> (1)

The main ingredients for the cost function are the geometric residuals and the photometric
residuals. The geometric residual tells us how satisfied a local camera is with our placement
of the surfel in the 3D map. The photometric error looks how satisfied a local camera is with
the coloring of the corresponding surfel.

K is the set of all the keyframes and <a href="https://www.codecogs.com/eqnedit.php?latex=$S_k$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$S_k$" title="$S_k$" /></a> is the set of all surfels that have a corresponding
measurement in keyframe <a href="https://www.codecogs.com/eqnedit.php?latex=$k&space;\in&space;K$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$k&space;\in&space;K$" title="$k \in K$" /></a>.
Tukey’s bitweight and Huber robust loss function with weighting parameter 10 are used.
Those are mainly used for smoothing out the loss functions landscape, and those functions
are commonly used in SLAM methods.
<a href="https://www.codecogs.com/eqnedit.php?latex=$w_photo&space;=&space;10^{-2}$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$w_photo&space;=&space;10^{-2}$" title="$w_photo = 10^{-2}$" /></a> is a constant to make the geometric residual function more important
Both functions have <a href="https://www.codecogs.com/eqnedit.php?latex=$\sigma^{-1}$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\sigma^{-1}$" title="$\sigma^{-1}$" /></a> constant to count for standard deviations of our geometric and
photometric measurements.

### 4.1 Geometric Residual

<a href="https://www.codecogs.com/eqnedit.php?latex=residual_{geom}(s,k)&space;=&space;(T_G^kn_s)^{T}(\pi^{-1}_{D,k}(\pi_{D,k}^{k}(T_G^{k}p_s))-&space;T_G^{k}p_s))\label{fun_geo}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?residual_{geom}(s,k)&space;=&space;(T_G^kn_s)^{T}(\pi^{-1}_{D,k}(\pi_{D,k}^{k}(T_G^{k}p_s))-&space;T_G^{k}p_s))\label{fun_geo}" title="residual_{geom}(s,k) = (T_G^kn_s)^{T}(\pi^{-1}_{D,k}(\pi_{D,k}^{k}(T_G^{k}p_s))- T_G^{k}p_s))\label{fun_geo}" /></a> (2)

To keep track of what the geometric residual is doing it’s important to keep in mind what
all the transformations are doing. We have the following dimensions:

- One 3D world space
- N 3D local camera spaces for N cameras
- N 2D local camera spaces for N cameras.

Each camera has a transformation matrix <a href="https://www.codecogs.com/eqnedit.php?latex=$T_G^k$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$T_G^k$" title="$T_G^k$" /></a> (often called extrinsic matrix) which transforms
coordinates from the world space to it’s local camera space.
Each camera has a projection function <a href="https://www.codecogs.com/eqnedit.php?latex=$\pi$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\pi$" title="$\pi$" /></a> (which is determined by the intrinsic matrix) which
transforms coordinates from the 3D local camera space to it’s 2D local camera space (depth
image).
Each camera has also an unprojection function <a href="https://www.codecogs.com/eqnedit.php?latex=$\pi^{-1}$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\pi^{-1}$" title="$\pi^{-1}$" /></a> (also based on the intrinsic matrix) which
transforms coordinates from the 2D local camera space to the 3D local camera space.

This is a difference vector in 3D space <a href="https://www.codecogs.com/eqnedit.php?latex=$(\pi^{-1}_{D,k}(\pi_{D,k}^{k}(T_G^{k}p_s))-&space;T_G^{k}p_s)$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$(\pi^{-1}_{D,k}(\pi_{D,k}^{k}(T_G^{k}p_s))-&space;T_G^{k}p_s)$" title="$(\pi^{-1}_{D,k}(\pi_{D,k}^{k}(T_G^{k}p_s))- T_G^{k}p_s)$" /></a>. Where the 3D center point
of the surfel in world space goes through the following transformations:


- From 3D world space to 3D local camera space by <a href="https://www.codecogs.com/eqnedit.php?latex=$T_G^k$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$T_G^k$" title="$T_G^k$" /></a>
- From 3D local camera space to 2D local camera space (depth image) by <a href="https://www.codecogs.com/eqnedit.php?latex=$\pi$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\pi$" title="$\pi$" /></a>
- From 2D local camera space (depth image) back to 3D local camera space by <a href="https://www.codecogs.com/eqnedit.php?latex=$\pi^-1$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\pi^-1$" title="$\pi^-1$" /></a>

The 3D center point of the surfelpsgoes through all our transformations, if all the parameters
of our transformations and the placement of our modeled surfel are correct, then difference
vector should be close to zero.

A difference/residual vector is a must have for SLAM and is also used in indirect methods.
What is really different from the indirect approach is the inclusion of the part (TGkns)Twhich
makes equation (2) a dot product. Equation (2) is a dot product between the 3D normal
vector of our modelled surfel in the 3D local camera space and a residual/difference vector
in the 3D local camera space. The dot product enforces the optimization scheme to show
preference to transformation parameters that move the surfel along the normal direction. A
dot product attains its lowest value between two vectors which are perpendicular. As long
as the surfel keeps moving in the normal direction, then the surfel and the normal vector will
still be perpendicular, and thus result into a low error.

### 4.2 Photometric Residual

<a href="https://www.codecogs.com/eqnedit.php?latex=residual_{photo}(s,k)&space;=&space;\left&space;\|&space;\begin{bmatrix}&space;I(\pi_{I,k}(s_1))&space;-&space;I(\pi_{I,k}(p_s))&space;\\&space;I(\pi_{I,k}(s_2))&space;-&space;I(\pi_{I,k}(p_s))&space;\end{bmatrix}&space;\right&space;\|_2&space;-&space;d_s" target="_blank"><img src="https://latex.codecogs.com/gif.latex?residual_{photo}(s,k)&space;=&space;\left&space;\|&space;\begin{bmatrix}&space;I(\pi_{I,k}(s_1))&space;-&space;I(\pi_{I,k}(p_s))&space;\\&space;I(\pi_{I,k}(s_2))&space;-&space;I(\pi_{I,k}(p_s))&space;\end{bmatrix}&space;\right&space;\|_2&space;-&space;d_s" title="residual_{photo}(s,k) = \left \| \begin{bmatrix} I(\pi_{I,k}(s_1)) - I(\pi_{I,k}(p_s)) \\ I(\pi_{I,k}(s_2)) - I(\pi_{I,k}(p_s)) \end{bmatrix} \right \|_2 - d_s" /></a> (3)

<a href="https://www.codecogs.com/eqnedit.php?latex=$\pi_{I,k}$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\pi_{I,k}$" title="$\pi_{I,k}$" /></a> is the projection function to the RGB space and I() outputs a intensity value at a certain
pixel location by using the bilinearly interpolation technique. <a href="https://www.codecogs.com/eqnedit.php?latex=$s_1$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$s_1$" title="$s_1$" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=$s_2$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$s_2$" title="$s_2$" /></a> are sampled points
within a surfel such that <a href="https://www.codecogs.com/eqnedit.php?latex=$s_1&space;-&space;p_s$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$s_1&space;-&space;p_s$" title="$s_1 - p_s$" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=$s_2&space;-&space;p$_s" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$s_2&space;-&space;p$_s" title="$s_2 - p$_s" /></a> are orthogonal (90 degree angle).

For the photometric error we need to define a function which tells us how far off our modeled
surfel colors are. The paper uses equation (3) which compares the scalar magnitude of two
gradients obtained from the RGB space; those magnitudes are the surfel visual descriptords,
every surfel has adsproperty, it’s one of the surfel properties from section 3..
The 2D gradient vector of a pixel points in the direction of the highest image intensity
change. The magnitude is the length of this vector which indicates how strong the intensity
change is. You could compute an average magnitude for all the pixels of a surfel. This might
be intuitively simple and logically to do. However, this not an efficient method for direct
SLAM. That’s why they just sample two points within the surfel and use those to determine
the gradient magnitude of the surfel.


## 5 Optimization

Figure 6 shows the optimization steps of their implemented direct BA scheme. The key
elements of the algorithm is the alternating behavior optimization in steps 4 and 6 which
deals with solving the geometric/photometric and keyframes parameters respectively.

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/optimization.png)

### 5.1 Surfel Creation

The keyframes are first partitioned in 4x4 pixel grids; these grids are used for surfel creation,
if there isn’t a single pixel within the grid which correspond to a surfel, then they randomly
chose one depth measurement within the grid to create a new surfel.
In this step there is also a filtering pixel step. it is based on a corresponding measurements count and an occlusion count. Where a good pixel should have a high corresponding
measurement count and a low occlusion count.

### 5.2 Surfel Normal

The surfel normal updates are not touched by the back-end BA instead they take an averaging
approach. A surfel is seen by a certain amount of k keyframes. Each keyframe has a gradient
obtained from the 2D depth map and transformed back to the 3D world space. Those k
gradients are averaged and this value will be the new normal vector property for the surfel.
The biggest benefit of this approach is that BA optimiation becomes less complicated, and
results into less computation time.

### 5.3 Surfel Position and Descriptors

The 3D surfel center position and the magnitude gradient color descriptors properties are
optimized by the BA by the Gauss-Newton (GN) procedure. At the same time we keep the
parameters for the keyframe poses fixed. This is why it’s called an alternating approach.
Normally, BA optimizes all parameters in one go.

### 5.4 Keyframe Pose

The pose defined by rotation and translation (rigid body motion), also called extrinsic matrix of a keyframe/camera. Can be compactly parametrized by a 6D vector and there is a
transformation to go back from the 6D vector to the matrix it represents also called exponential mapping (fact from Lie Algebra). This vector is optimized by the BA with GN for
each keyframe, and by the alternating approach we now fix the parameters for the surfels
properties in the BA scheme.


## 6 Benchmarks

### 6.1 TUM RGB-D

The indirect methods are BundleFusion [3] — uses SIFT as feature descriptor — and ORB-
SLAM2 [4]. While the other methods lean more to the direct approach.

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/bench_1.png)

The TUM RGB-D benchmark contains multiple datasets for testing the accuracy of the
SLAM pipeline. The BAD-SLAM approach reaches the second average rank once they also
optimize for the intrinsics. Once we have to fix the intrinsics the performance degrades quite
a bit for more difficult datasets. You should always try to optimise your intrinsic in the SLAM
setting, then aspects like distortions are better modeled. However, with dynamic scenes the
optimization can be computationally infeasible; in such a case the intrinsics should be fixed
by experimentation. The direct methods use all the information in the image. Therefore, if
the intrinsics can’t be optimized and aren’t correct according to ground-truth, it uses more
wrong information than the indirect methods which only use a subspace of the image plane.

They show only results for a small subset of the TUM benchmark; it would be interesting to
see how this methods performs on more datasets from the TUM benchmarks.


### 6.2 Own benchmark

Before we go on to the result, I want to briefly discuss the hardware side of things, because
The previous benchmark doesn’t fulfill the hardware assumptions the authors of BAD SLAM
made. Therefore, they made their own dataset to show the full potential of their method.
SLAM is a practical problem and hardware can make a significant difference in performance
for the algorithms at the software side. To create their dataset they used a synchronised
global shutter cameras with an Asus Xtion Live Pro. Whereas the TUM dataset is obtained
by asynchronous rolling shutter cameras.

#### 6.2.1 Global shutter vs rolling shutter

These are techniques how the camera sensors captures light of the scene. These techniques
are very important for the possible range of images we can capture in the physical world.
The difference between those techniques are most apparent for difficult fast moving scenes.

Rolling shutter does not capture images at the same time. It sequentially row/column based
scans the image. Let’s say you use row based scanning then every row captures light of the
physical scene at a different time. This also creates the asynchronous part; each row of the
captured images is taken at a different timestamp.
Global shutter captures all the light of the scene at the same time, thus all parts of the
sensors are either on or off. The following link has multiple gif’s and videos which show the
effects nicely: shutter visualizations
Global shutter has less image artifacts than rolling shutter, but the electronic circuits are
more complicated which makes global shutter cameras more expansive than rolling shutter
cameras. Also, a lot of artifact’s like distortion and skewness can be dealt with at the software
side and if your reading speed is fast enough then the artifacts are hard to notice.


#### 6.2.2 Benchmark results

![alt text](https://github.com/BigSorry/Blogs/blob/master/images/bench_2.png)

In this method, BAD SLAM is the best approach and even outperforms ORB-SLAM2. While
this definitely is impressive for a direct method; their method is fine tuned for this dataset
and the authors do not explain how they fine tuned the other methods for this dataset.
Good performance in one dataset is not enough for generalization, thus testing it on different
datasets with different hardware characteristics will really give us insights how good this
method really is. As of yet, we can only be conservative and say that their method looks
promising and is worthwhile to test it on more datasets.

## 7 Last Remarks

The authors proposed a RGB-D SLAM method with real time direct BA using surfels. They
believe rolling shutter, asynchronous RGB-D frames, and depth distortion are better to be
solved in hardware than software. They also created a dataset which fullfill their hardware
requirements.

+ Real time direct SLAM method
+ Created a new dataset with new challenges for SLAM methods
+ Good performance on the few datasets from the benchmarks

- Hard to pinpoint what exactly is novel about the paper. However, to achieve real time
direct SLAM with good performance a lot of optimization trade offs must be made; do we
include gradient normal in BA or not, hyperparameters in the cost function, how to sample
the gradient magnitudes. This is something you can’t really avoid in direct SLAM.
- Tested only on few datasets


# References

[1] Richard Hartley and Andrew Zisserman. Multiple view geometry in computer vision.
Cambridge university press, 2003.

[2] Thomas Schops, Torsten Sattler, and Marc Pollefeys. Bad slam: Bundle adjusted direct
rgb-d slam. InProceedings of the IEEE Conference on Computer Vision and Pattern
Recognition, pages 134–144, 2019.

[3] Angela Dai, Matthias Nießner, Michael Zollh ̈ofer, Shahram Izadi, and Christian Theobalt.
Bundlefusion: Real-time globally consistent 3d reconstruction using on-the-fly surface
reintegration.ACM Transactions on Graphics (ToG), 36(3):24, 2017.

[4] Raul Mur-Artal and Juan D Tard ́os. Orb-slam2: An open-source slam system for monoc-
ular, stereo, and rgb-d cameras.IEEE Transactions on Robotics, 33(5):1255–1262, 2017.


