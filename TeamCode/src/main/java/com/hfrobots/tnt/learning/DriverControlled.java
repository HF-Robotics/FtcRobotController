/*
 Copyright (c) 2021 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

package com.hfrobots.tnt.learning;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// To start, you will copy this file to a new file, and change the name of this Tele-Op to have
// your name in it.
@TeleOp(name = "Tele-op-learning")
@Disabled // Then, you'll remove this line, so that the OpMode will be loaded by the robot
          // controller, and be listed on the drivers' station
public class DriverControlled extends OpMode {

    private Drivebase drivebase;

    private NinjaGamePad driverGamepad;

    // The code inside init(), will run when "Init" has been pressed on the drivers' station.
    // Usually, we setup all of the hardware, and code that is related to the gamepads here.
    @Override
    public void init() {
        // This defines a drive base variable, that knows how to convert some requests from
        // your code in this file, into moving the drive base around.
        //
        // For a few seasons now, we have had a convention on what motors for a drive base are
        // named in the hardware map, and where they are located on the robot. Because of this,
        // this TeleOp code should work with pretty much any robot the team has on hand.
        drivebase = new Drivebase(hardwareMap);

        // TNT has some code that makes the gamepads easier to use - this code converts the
        // gamepad object that is provided by FIRST, into our nicer-to-use one.
        driverGamepad = new NinjaGamePad(gamepad1);
    }

    // Once "Start" has been pressed on the drivers' station, this code will be called
    // over and over until "Stop" has been pressed. Here is where you will add code to make
    // the robot (in this case, the drive base) do things.

    @Override
    public void loop() {

        // To make the robot move, you're going to need to add some new code, in the following
        // order:

        // (1) First, read values from the game pad. You decide which ones you want to use,
        //     but note that the best kind are the ones that come from these:
        //
        // driverGamepad.getLeftStickX().getPosition();
        // driverGamepad.getLeftStickY().getPosition();
        // driverGamepad.getRightStickX().getPosition();
        // driverGamepad.getRightStickY().getPosition();
        //
        // The values from these controls have a range of (-1, 1), which also is the same
        // as the power levels for the motors on our robot.

        // (2) The next step is give the computed power values to each motor in the
        // drive base separately, that looks like this, you'll use the values you read above,
        // and compute power levels for each motor that drives a wheel (leftFrontPower,
        // leftRearPower, rightFrontPower, rightRearPower).
        //
        // You may want to check out this section in Game Manual 0 to understand the concept(s)
        // https://gm0.org/en/stable/docs/software/mecanum-drive.html#deriving-mecanum-control-equations
        //
        // Hint - it's possible to treat a Mecanum drive base like a tank drive if you set
        // both left motors to the same power level, and both right motors to the same power level
        //

        double leftFrontPower = 0;
        double leftRearPower = 0;
        double rightFrontPower = 0;
        double rightRearPower = 0;

        drivebase.setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }
}
