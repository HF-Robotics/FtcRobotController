/*
 Copyright (c) 2025 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2526;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.RangeInput;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WheeledLauncher implements PeriodicTask {
    // These constants are the tunables for our launcher.
    protected static final double KICKER_SERVO_RAISED_POSITION =.775;

    protected static final double KICKER_SERVO_LOWERED_POSITION = .6;

    private static final double FAR_LAUNCH_VELOCITY = 1500;

    public static final int FAR_HOOD_POSITION = 273;

    private static final double MEDIUM_LAUNCH_VELOCITY = 1210;

    public static final int MEDIUM_HOOD_POSITION = 0;

    private static final double CLOSE_LAUNCH_VELOCITY = 1050;

    public static final int CLOSE_HOOD_POSITION = 0;

    // END: Tunables

    // These are the components of our launcher
    protected final DcMotorEx launcherMotor;

    protected final Servo kickerServo;

    private double requestedVelocity = 0;

    private HoodController hoodController;

    private final Telemetry telemetry;

    private final AprilTagAligner aprilTagAligner;

    public WheeledLauncher(final HardwareMap hardwareMap,
                           final Telemetry telemetry,
                           final Ticker ticker,
                           AprilTagAligner aprilTagAligner) {
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        this.aprilTagAligner = aprilTagAligner;
        launcherMotor.setVelocityPIDFCoefficients(135,3, 0, 14);
        this.telemetry = telemetry;
        kickerServo.setPosition(KICKER_SERVO_LOWERED_POSITION);
        setupHoodController(hardwareMap, telemetry, ticker);
    }

    protected void setupHoodController(final HardwareMap hardwareMap,
                                       final Telemetry telemetry,
                                       final Ticker ticker) {
        hoodController = new HoodController(hardwareMap, telemetry, ticker);
    }

    public void setHoodAngleAdjust(final RangeInput throttle) {
        if (hoodController != null) {
            hoodController.setManualControl(throttle);
        }
    }

    public void homeHood() {
        if (hoodController != null) {
            hoodController.goHome();
        }
    }

    public void autoRange() {
        if (aprilTagAligner == null) {
            return;
        }


        final Double measuredRange = aprilTagAligner.getRange();

        if (measuredRange == null) {
            return;
        }

        final double adjustedRangeInches = measuredRange - 6;
        final double wheelSpeed;
        final int hoodPosition;

        if (adjustedRangeInches < 0) {
            wheelSpeed = 920;
            hoodPosition = 0;
        } else if (adjustedRangeInches < 12) {
            wheelSpeed = 920;
            hoodPosition = 0;
        } else if (adjustedRangeInches < 24) {
            wheelSpeed = 1100;
            hoodPosition = 0;
        } else if (adjustedRangeInches < 36) {
            wheelSpeed = 1125;
            hoodPosition = 0;
        } else if (adjustedRangeInches < 48) {
            wheelSpeed = 1210;
            hoodPosition = 0;
        } else if (adjustedRangeInches < 60){
            wheelSpeed = 1270;
            hoodPosition = 178;
        } else {
            wheelSpeed = 1355;
            hoodPosition = 252;
        }

        // Set launch velocity
        maybeSetLaunchVelocity(wheelSpeed);
        // Set hood position

        hoodController.setDynamicPosition(hoodPosition);
    }

    public boolean isHoodIdle() {
        if (hoodController != null) {
            return hoodController.isIdle();
        }

        return true;
    }

    public void adjustVelocity(final RangeInput adjustmentThrottle) {
        double adjustmentAmount = -adjustmentThrottle.getPosition();

        if (requestedVelocity < 0) {
            return;
        }

        if (adjustmentAmount != 0) {
            maybeSetLaunchVelocity(requestedVelocity + (adjustmentAmount * 4.0));
        }
    }

    public void closeLaunchVelocity() {
        maybeSetLaunchVelocity(CLOSE_LAUNCH_VELOCITY);

        if (hoodController != null) {
            hoodController.setPosition(TargetDistance.CLOSE);
        }
    }

    public void mediumLaunchVelocity() {
        maybeSetLaunchVelocity(MEDIUM_LAUNCH_VELOCITY);

        if (hoodController != null) {
            hoodController.setPosition(TargetDistance.MEDIUM);
        }
    }

    public void farLaunchVelocity() {
        maybeSetLaunchVelocity(FAR_LAUNCH_VELOCITY);

        if (hoodController != null) {
            hoodController.setPosition(TargetDistance.FAR);
        }
    }

    public void stopLauncher() {
        maybeSetLaunchVelocity(0);
    }

    public boolean isMoving() {
        return launcherMotor.getVelocity() > 50;
    }

    public boolean isAtTargetVelocity() {
        double currentVelocity = launcherMotor.getVelocity();

        return Math.abs(requestedVelocity - currentVelocity) < 100;
    }

    public void lowerKicker() {
        kickerServo.setPosition(KICKER_SERVO_LOWERED_POSITION);
    }

    public void safelyRaiseKicker() {
        final boolean isLauncherWheelMoving = isMoving();
        final boolean isAtTargetVelocity = isAtTargetVelocity();

        if (isLauncherWheelMoving && isAtTargetVelocity) {
            kickerServo.setPosition(KICKER_SERVO_RAISED_POSITION);
        } else {
            Log.i(LOG_TAG, "Not safe to raise kicker: moving: " + isLauncherWheelMoving + ", at velocity: " + isAtTargetVelocity);
        }
    }

    public void raiseKickerNoMatterWhat() {
        kickerServo.setPosition(KICKER_SERVO_RAISED_POSITION);
    }

    private void maybeSetLaunchVelocity(final double velocity) {
        if (requestedVelocity != velocity) {
            requestedVelocity = velocity;
            launcherMotor.setVelocity(requestedVelocity);
        }
    }

    @Override
    public void periodicTask() {
        String currentVelocityIndicator = (isAtTargetVelocity() ? "*" : "") + launcherMotor.getVelocity();
        telemetry.addData("Launcher",  "req_vel %s - cur_vel %s",
                Double.toString(requestedVelocity),
                currentVelocityIndicator);

        if (hoodController != null) {
            hoodController.periodicTask();
        }
    }

    public enum TargetDistance {
        CLOSE, MEDIUM, FAR
    }
}