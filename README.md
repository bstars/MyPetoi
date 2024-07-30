A simple inverse kinematics and gaits for Petoi Bittle robot. 

MyPetoi.pdf briefly shows the predefined gaits and inverse kinematics.

MyPetoi.ino is the robot side code, it simply receives target angles from python script.

petoi_kinematics.py implements forward and inverse kinematics, and a simple matplotlib rendering.

petoi_simulate.py tends to control the matplotlib robot with cmd/UI, but it does not work due to threading issues.

petoi_control.py implements a cmd and a UI controller that controls the physical robot directly.

<p>
  <img width="35%" src="https://github.com/bstars/MyPetoi/blob/main/MyPetoi.gif">
</p>
