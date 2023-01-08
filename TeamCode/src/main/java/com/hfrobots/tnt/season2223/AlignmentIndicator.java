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

package com.hfrobots.tnt.season2223;

import com.hfrobots.tnt.corelib.sensors.RevLedIndicator;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class AlignmentIndicator {
    private final Servo leftPointerServo;

    private final Servo rightPointerServo;

    private final static double LEFT_POINT_POSITION = .4;

    private final static double RIGHT_POINT_POSITION = .6;

    private final static double MAX_RANGE_MM = 150;

    private final static double GRAB_MAX_RANGE_MM = 100;

    private final static double GRAB_MIN_RANGE_MM = 40;

    private final Gamepad driverGamepad;

    private final Gamepad operatorGamepad;

    public AlignmentIndicator(final HardwareMap hardwareMap, Gamepad driverGamepad, Gamepad operatorGameped) {
        leftPointerServo = hardwareMap.get(Servo.class, "leftPointerServo");
        rightPointerServo = hardwareMap.get(Servo.class, "rightPointerServo");

        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGameped;

        pointServosCenter();
    }

    public void pointServosCenter() {
        leftPointerServo.setPosition(.5);
        rightPointerServo.setPosition(.5);
    }

    public void distancesToIndicators(final ConeLocalizer.ConeDistances coneDistances) {
        if (!coneDistances.isDistanceSensorsWorking()) {
            indicateBrokenSensors();

            return;
        }

        double leftDistanceMm = coneDistances.getLeftDistanceMm();
        double rightDistanceMm = coneDistances.getRightDistanceMm();
        double centerDistanceMm = coneDistances.getCenterDistanceMm();

        double absoluteDifferenceMm = Math.abs(rightDistanceMm - leftDistanceMm);

        if (centerDistanceMm > MAX_RANGE_MM) {
            pointServosCenter();

            return;
        }

        if (absoluteDifferenceMm > 10) {
            if (rightDistanceMm > leftDistanceMm) {
                // indicate right

                pointRight(absoluteDifferenceMm);

            } else if (rightDistanceMm < leftDistanceMm) {
                // indicate left
                pointLeft(absoluteDifferenceMm);

            }
        } else {
            pointServosCenter();

            if (centerDistanceMm <= GRAB_MAX_RANGE_MM && centerDistanceMm >= GRAB_MIN_RANGE_MM) {
                driverGamepad.rumbleBlips(1);
                operatorGamepad.rumbleBlips(2);
            }
        }
    }

    private void pointLeft(double absoluteDifferenceMm) {
        double differenceFromCenter =  0.5 - LEFT_POINT_POSITION;
        double positionRange = differenceFromCenter / 40;
        double position = 0.5 - absoluteDifferenceMm * positionRange;

        if (position < LEFT_POINT_POSITION) {
            position = LEFT_POINT_POSITION;
        }

        rightPointerServo.setPosition(position);
        leftPointerServo.setPosition(position);
    }

    private void pointRight(double absoluteDifferenceMm) {
        double differenceFromCenter =  RIGHT_POINT_POSITION - .5;
        double positionRange = differenceFromCenter / 40;
        double position = 0.5 + absoluteDifferenceMm * positionRange;

        if (position > RIGHT_POINT_POSITION) {
            position = RIGHT_POINT_POSITION;
        }

        rightPointerServo.setPosition(position);
        leftPointerServo.setPosition(position);
    }

    private void indicateBrokenSensors() {

    }

    private void doIndicator(final double distanceMm,
                             final RevLedIndicator indicator) {
        if (distanceMm < 50) {
            indicator.setColor(RevLedIndicator.LedColor.GREEN);
        } else if (distanceMm < 100) {
            indicator.setColor(RevLedIndicator.LedColor.AMBER);
        } else if (distanceMm < 300) {
            indicator.setColor(RevLedIndicator.LedColor.RED);
        } else {
            indicator.setColor(RevLedIndicator.LedColor.OFF);
        }
    }
}
