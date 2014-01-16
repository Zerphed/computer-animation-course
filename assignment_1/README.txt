Name: Joonas Nissinen
Student number: ******

List of used external libraries:
--------------------------------
None.


One line command for compiling:
--------------------------------
make

I didn't have time to try the final solution in Paniikki so I hope it compiles the same way it does in my OSX. 
I also had some problems parsing some of the BVH files. I think this might be due to variation in the line endings. I sincerely hope this doesn't cause a point reduction.
If you need it fixed please contact me and I'll try to look into it further.


Explanation:
-------------
The program parses the BVH file and represents the information as a stick figure using cylinders and spheres. You can switch between different interpolation methods from the "Interpolation method" -menu. The default one is the most pleasing slerp interpolation. 

* None: No interpolation for anything.
* Lerp: Lerp interpolates only the position and does nothing for the rotations.
* Slerp: Slerp interpolates the rotations using slerp and does lerp interpolation for the positions.
* Euler: Euler interpolates the rotations as Euler angles and does lerp interpolation for the positions.

The program supports mandatory camera control, but due to the OSG library's default control settings controlling the model can be difficult at times. The camera supports rotating around the model by dragging. Zooming in and out requires holding down CTRL and moving the mouse when pressed. You can position the model by pressing ALT and using pressed mouse control.


List of implemented features:
------------------------------
In my opinion I implemented all but the lerp functionality of the required features, however the side effects of some of the methods can be hard to notice.

* Reading and showing the motion as such (while dropping extra frames or showing a frame more the once to preserve the timing), 1 point
* Linear interpolation of the rotations as the Euler angles that shows the side effect of the gimbal lock, 1 point
* Slerp interpolation of rotations as quaternions (this should not produce artifacts) 1 point
* Visualization of the trajectories of body parts, 1 point
