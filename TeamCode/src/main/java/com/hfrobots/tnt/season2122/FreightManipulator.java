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

package com.hfrobots.tnt.season2122;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import lombok.NonNull;

/**
 * The class that makes the mechanism that picks up and delivers freight, work.
 */
public class FreightManipulator {
    // (1) We need to add the motors, servos and sensors this mechanism will use first, they go
    // in this location in the file. The mechanism requirements document can be consulted to
    // figure out what these are.

    final DcMotorEx armMotor;

    public static final double LEFT_FULL_POSITION = 0.0D;

    public static final double RIGHT_FULL_POSITION = 1.0D;

    public static final double CENTER_POSITION = 0.5D;

    public static final double LEFT_GRIPPER_OPEN_POSITION = CENTER_POSITION;

    public static final double LEFT_GRIPPER_CLOSED_POSITION = LEFT_FULL_POSITION;

    public static final double RIGHT_GRIPPER_OPEN_POSITION = CENTER_POSITION;

    public static final double RIGHT_GRIPPER_CLOSED_POSITION = RIGHT_FULL_POSITION;

    public static final double SPEED_DIVISIOR = 10D;

    public static final int ARM_SAFE_POSITION_DIFFERENCE = 20;

    public final int armMotorStartingPosition;

    final Servo leftGripperServo;

    final Servo rightGripperServo;

    public FreightManipulator(@NonNull final HardwareMap hardwareMap) {

        // (2) Here is where we setup the items in (1), by finding them in the HardwareMap
        //
        // If this mechanism will have its own automation through a state machine,
        // this would also be setup here.

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorStartingPosition = armMotor.getCurrentPosition();

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftGripperServo = hardwareMap.get(Servo.class, "leftGripperServo");
        rightGripperServo = hardwareMap.get(Servo.class, "rightGripperServo");
    }

    // (3) Here, we define what the mechanism does, by adding methods. These methods contain
    // instructions for what to do with the items listed in step (1).

    public void moveArmUp(double speed) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(-speed / SPEED_DIVISIOR);
    }

    public void moveArmDown(double speed) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(speed / SPEED_DIVISIOR);
    }

    public void stopArm() {
        // Feed Forward
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(-.25);
    }

    public void moveToSafePosition() {
        armMotor.setTargetPosition(armMotorStartingPosition + ARM_SAFE_POSITION_DIFFERENCE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
    }

    public void openGripper() {
        leftGripperServo.setPosition(LEFT_GRIPPER_OPEN_POSITION);
        rightGripperServo.setPosition(RIGHT_GRIPPER_OPEN_POSITION);
    }

    public void closeGripper() {
        leftGripperServo.setPosition(LEFT_GRIPPER_CLOSED_POSITION);
        rightGripperServo.setPosition(RIGHT_GRIPPER_CLOSED_POSITION);
    }

    // (4) If this component needs to take action during every loop of tele-op or auto,
    // implement the PeriodicTask interface and its required methods, and write the
    // code that performs those actions in the periodicTask() method.

    // (5) If this mechanism will have automation using a state machine, we describe the
    // states as new classes here, and set them up in (2).
}
