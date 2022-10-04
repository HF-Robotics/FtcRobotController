The classes in the package in this directory are skeletons and examples of how the Tech Ninja
Team has historically built driver-controlled and autonomous robot programs. They are designed to
serve as inspiration at the beginning of each new season, and simpler code to use for teaching 
programming to new TNT team members.

There are two opmode templates for reference:

* AutoTemplate - a basic autonomous program that has alliance selection and route/program choice
but nothing else. The autonomous action by the robot is driven by a StateMachine. Template code
for states that perform common tasks (moving the robot, servos, etc) is in the ExampleStates class.

A new program based on this template should also have a copy of ExampleDriveConstants, and use
that during init() when setting up the drivebase. If the robot is very much different than 18x18 
with 4 AndyMark Mecanum wheels driven by Orbital 20 motors, then you will need to tune some of the 
parameters in that copy of ExampleDriveConstants. More information on that is available in the 
team's google drive (a PDF copy of learnroadrunner.com)

* DriverControlledTemplate - a basic tele-op program for a robot that uses a drive base with 
Mecanum wheels. It uses a DriverControls class with the team's "standard" controls, robot direction
on the left stick, rotation on the right stick, fast/slow ("party mode") on the left trigger,
inversion of controls on the right trigger, and bumpers for short strafing adjustments. 

