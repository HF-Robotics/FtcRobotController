/*
 Copyright (c) 2022 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.util.templates;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.ServoPositionState;
import com.ftc9929.corelib.state.State;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveBase;
import com.hfrobots.tnt.corelib.drive.mecanum.TrajectoryFollowerState;
import com.hfrobots.tnt.corelib.drive.mecanum.TurnState;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.concurrent.TimeUnit;

public class ExampleStates {
    // These should exist from template auto, if using RoadRunner:

    private Ticker ticker;

    private Telemetry telemetry;

    private RoadRunnerMecanumDriveBase driveBase;

    private Servo servo;

    //------------------------------------------------------------------------------
    //
    // Everything below here is an example we can use in autonomous...

    // A state that moves the robot in some direction
    //
    // Note that createTrajectory() is called when this state runs,
    // not when it is created, so you can alter the trajectory
    // based on data stored in fields in the Autonomous class like
    // alliance color, CV-detected things, etc.

    State moveRobot = new TrajectoryFollowerState("Move robot",
            telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
        @Override
        protected Trajectory createTrajectory() {
            TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

            // Move forwards some distance in inches
            trajectoryBuilder.forward(10);

            // Move backwards some distance in inches
            trajectoryBuilder.back(10);

            // Strafe left some distance in inches
            trajectoryBuilder.strafeLeft(27 + 3.5 + 3);

            // Strafe right some distance in inches
            trajectoryBuilder.strafeRight(27 + 3.5 + 3);

            // Spline (forward/back and left/right in continuous path) to relative position
            // (careful, RoadRunner's X axis is our Y axis)
            trajectoryBuilder.splineTo(new Vector2d(0, 0), 0);

            return trajectoryBuilder.build();
        }
    };

    // Turn the robot (90 degrees clockwise)

    State turnNinetyDegreesClockwise = new TurnState("turn 90 deg clockwise",
            telemetry, new Turn(Rotation.CW, 90), driveBase, ticker,
            TimeUnit.SECONDS.toMillis(20 * 1000));

    // Set a servo to a given position (this does not wait any amount of time)

    State servoState = new ServoPositionState("move servo", telemetry, servo, 0.5);

    // Run any Java code as a state

    State runnableState = new RunnableState("Run some code", telemetry, () -> {
        // Your code here
    });
}
