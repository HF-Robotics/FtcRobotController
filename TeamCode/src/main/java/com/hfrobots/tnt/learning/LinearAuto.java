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

import com.google.common.base.Stopwatch;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Linear-Auto-learning")
@Disabled
public class LinearAuto extends LinearOpMode {
    private DcMotorEx rightFrontDriveMotor;
    private DcMotorEx rightRearDriveMotor;
    private DcMotorEx leftFrontDriveMotor;
    private DcMotorEx leftRearDriveMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // For this lesson, we will not use the DriveBase class, but work with the DcMotors
        // in the drive base, directly, here is how we map them from the hardware map/control hub
        // to variables your code can talk to:

        rightFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor");
        rightRearDriveMotor = hardwareMap.get(DcMotorEx.class, "rightRearDriveMotor");
        leftFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor");
        leftRearDriveMotor = hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor");

        // We always want "forward" power to be the value '1', the motors on opposite sides of
        // the robot rotate in opposite directions because of their geometry, so we reverse the
        // direction of one side here:

        rightFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Past this point, the code will run step by step until the end of this method. If the
        // robot needs to do something repetitively, or until it reaches some condition, then
        // the code needs to have a loop, but that loop must always check opModeIsActive() too!
        // It looks something like this:

        // FIXME: Pick one of these to run at a time:

        //doDeadReckoning();

        //doBangBang();

        //doPid();
    }

    void doDeadReckoning() {
        // Dead reckoning
        //
        // This method does not require the robot to "know where it is" on the field.
        //
        // It only knows that it has so far to go. If it goes so fast, for some amount of time, it
        // will get there:
        //
        // Drive full speed for 3 feet
        //
        // Robot speed = (approx) 4.5 feet/sec
        //
        // This will take ??? seconds
        //
        // What will go wrong? :)

        // (1) We need a way to measure elapsed time, let's use a Stopwatch

        final Stopwatch stopwatch = Stopwatch.createUnstarted();

        final int elapsedTimeMilliseconds = 0; // FIXME: Need to compute this from what we know

        // (2) Set the motor powers all to '1'

        // FIXME: Actually set the motor powers all to the value 1

        // (3) Wait until the time has elapsed

        stopwatch.start();

        // Wait until the elapsed time passes

        // FIXME: This loop might be missing something that is required in linear op modes!

        while (opModeIsActive() && stopwatch.elapsed(TimeUnit.MILLISECONDS) < elapsedTimeMilliseconds);

        // (4) Stop the motors by setting all of their power to 0.

        // FIXME: Stop all of the motors by setting their power to 0

        // Observation - run this a few times once you get it working. You may have to adjust
        //               the time and/or power levels to get it working.
        //
        //               How repeatable are the distances traveled, what is the error?

    }

    void doBangBang() {
        // "Bang Bang" introduces the concept of an encoder - so the robot can know how far
        // it has traveled, based on the number of rotations of the motor. The encoder is also
        // known in control language as "feedback", something that measures the outside world.
        //
        // Step one is to calculate the "goal" which is the number of encoder counts needed to
        // travel the distance.
        //
        // Then, you ask the robot to do this until the goal is reached:
        //
        //       if short of the goal, drive full power forwards towards it.
        //
        //       if past the goal, drive full power back towards it.
        //
        //       if at the goal, stop. (special note on this one)
        //
        // If we want to convert distance traveled to encoder count, you need to divide the
        // distance by the circumference of the wheels, and then multiply that by encoder count
        // for a full revolution of the motor.
        //
        // Our robot wheels are 4", what is their circumference?
        //
        // The motors on most of our robots are
        //
        // So, let's get started:

        double ticksPerRev = 537.6; // Neverest 20 orbital has this many ticks/revolution
        double distanceInEncoderCounts = 36 /* inches */ / 1 /* FIXME: use real circumference */ * ticksPerRev;

        // DcMotors don't usually start with their encoder count at 0. It certainly won't be there
        // if the robot has moved beforehand. Therefore, we need to calculate the "target" or
        // "goal" encoder count.
        //
        // To keep things simple for this lesson, pick one motor only to use as the encoder
        // feedback to the control algorithm.

        int currentEncoderCount = leftFrontDriveMotor.getCurrentPosition();

        // (1) You now know the current position of one motor, and how many encoder ticks/counts
        //     it should take to move the distance of 3 feet. Calculate the "goal" encoder count

        double goalEncoderCount = 0; // FIXME: Calculate this value

        // (2) Now, address the FIXMEs inside the while loop, load the program onto the robot and
        //     see what happens when it runs.
        //
        //     We'll come back to try and address some of the problems you will see with this
        //     approach

        while (opModeIsActive()) {
            currentEncoderCount = leftFrontDriveMotor.getCurrentPosition();

            if (currentEncoderCount < goalEncoderCount) {
                // FIXME: Drive forward by setting all motor powers to 1.0
            } else if (currentEncoderCount > goalEncoderCount) {
                // FIXME: Drive backwards by setting all motor powers to -1.0
            } else {
                // FIXME: Stop all motors

                break; // exit the loop
            }
        }

        // FIXME: Op-mode has stopped. Set all motors
        //        to '0' power to stop the robot
    }

    void doPid() {
        // PID is Proportional, Integral, Derivative control. It aims to get to the target
        // as quickly as possible, without over-shooting or oscillating
        // (going back and forth). Getting there does take some tuning, though.
        //
        // Just like "bang bang", step one is to calculate the "goal" which is
        // the number of encoder counts needed to travel the distance.
        //


        // If we want to convert distance traveled to encoder count, you need to divide the
        // distance by the circumference of the wheels, and then multiply that by encoder count
        // for a full revolution of the motor.
        //
        // Our robot wheels are 4", what is their circumference?
        //
        // The motors on most of our robots are Neverest 20's, with 537.6 encoders/revolution
        //
        // So, let's get started:

        double ticksPerRev = 537.6; // Neverest 20 orbital has this many ticks/revolution
        double distanceInEncoderCounts = 36 /* inches */ / 1 /* FIXME: use real circumference */ * ticksPerRev;

        // DcMotors don't usually start with their encoder count at 0. It certainly won't be there
        // if the robot has moved beforehand. Therefore, we need to calculate the "target" or
        // "goal" encoder count.
        //
        // To keep things simple for this lesson, pick one motor only to use as the encoder
        // feedback to the control algorithm.

        int currentEncoderCount = leftFrontDriveMotor.getCurrentPosition();

        // (1) You now know the current position of one motor, and how many encoder ticks/counts
        //     it should take to move the distance of 3 feet. Calculate the "target" encoder count

        double targetEncoderCount = 0; // FIXME: Calculate this value

        // (2) Now, address the FIXMEs inside the while loop, load the program onto the robot and
        //     see what happens when it runs.
        //
        //     We'll come back to try and address tuning the PID controller step by step
        //     and adding the I and D (not always needed) terms.

        PidController pidController = PidController.builder().setAllowOscillation(true)
                .setKp(0 /* FIXME */)
                //.setkI(0) FIXME - once we have a good kP, set kI
                //.setkD(0) FIXME - once we have a good kI - *maybe* set a kD
                .setTolerance(100 /* FIXME - what would be a good value here? */)
                .build();

        pidController.setTarget(targetEncoderCount, leftFrontDriveMotor.getCurrentPosition());

        while (opModeIsActive() && !pidController.isOnTarget()) {
            currentEncoderCount = leftFrontDriveMotor.getCurrentPosition();

            double motorPower = pidController.getOutput(currentEncoderCount);

            // FIXME: Send this power to all four motors
        }

        // FIXME: Either op-mode has stopped, or reached target, set all motors
        //        to '0' power to stop the robot
    }
}