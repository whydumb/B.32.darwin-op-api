# B.32.darwin-op-api


Daniele Tricoli
2:57 AM (8 hours ago)
to me

Hello and sorry for this late reply I was very busy. :(

I did not use Webots, I was aware of it but when I started working on DARwIn-OP the unit we have in lab had the mainbord broken, so I started from scratch ed invested on ROS.
I wrote few ROS nodes one of them to calculate forward kinematics, but my experiment was about self balancing a ball in a rail between DARwIn-OP hands.
So I did not tried to make it walk, but I wrote a generic system so it could be used.
For an already implemented walking module you can look at https://github.com/ROBOTIS-GIT/ROBOTIS-OP3
I did not tried it, but it should work since it's done by Robotis.

There is also this: https://www.generationrobots.com/en/content/83-carry-out-simulations-and-make-your-darwin-op-walk-with-gazebo-and-ros
It was the first thing I tried, but keep in mind that their URDF has wrong inertial matrices and center of mass¹, for this reason I fixed in the package I published it. So their wolking module works but it is as the gravity is not the real one.

I saw the video, first of all congratulation! But is the motion handled on Minecraft side? If yes maybe you have to look more how they handle it. If, instead, you can send joints and links position to minecraft (i don't know if you can only use joints since links are attached) you could have a ROS node where you can send the movement command and then you get the simulated position of joints. So I would focus on the side where the motion rendering is done first.

I don't know where to ask for help (of course you can write to me and I will help as I can)... maybe on reddit or discord there are some groups... I don't know honestly... 
