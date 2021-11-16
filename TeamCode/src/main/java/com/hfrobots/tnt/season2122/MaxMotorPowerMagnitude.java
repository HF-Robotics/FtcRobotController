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

import com.google.common.collect.ImmutableSet;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Set;

/**
 * Class that helps calculate the max absolute magnitude of motor power for a given set
 * of motors.
 */
public class MaxMotorPowerMagnitude {
    private final Set<DcMotorEx> motors;

    /**
     * Creates an instance using our standard motor naming pattern for a holonomic drivebase
     */
    public static MaxMotorPowerMagnitude forDrivebase(HardwareMap hardwareMap) {
        ImmutableSet<DcMotorEx> allDrivebaseMotors = ImmutableSet.of(
                hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor"),
                hardwareMap.get(DcMotorEx.class, "rightRearDriveMotor"),
                hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor"),
                hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor"));

        return new MaxMotorPowerMagnitude(allDrivebaseMotors);
    }

    public MaxMotorPowerMagnitude(Set<DcMotorEx> motors) {
        this.motors = motors;
    }

    public double getMaxPowerMagnitude() {
        double maxAbsolutePower = 0.0D;

        for (DcMotorEx motor : motors) {
            final double absolutePower = Math.abs(motor.getPower());

            if (absolutePower > maxAbsolutePower) {
                maxAbsolutePower = absolutePower;
            }
        }

        return maxAbsolutePower;
    }
}
