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

    private static final double FAR_LAUNCH_VELOCITY = 1598;

    private static final double MEDIUM_LAUNCH_VELOCITY = 1300;

    private static final double CLOSE_LAUNCH_VELOCITY = 1080;

    // These are the components of our launcher
    protected final DcMotorEx launcherMotor;

    protected final Servo kickerServo;

    private double requestedVelocity = 0;

    private HoodController hoodController;

    private final Telemetry telemetry;

    public WheeledLauncher(final HardwareMap hardwareMap,
                           final Telemetry telemetry,
                           final Ticker ticker) {
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
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
        final boolean isMoving = isMoving();
        final boolean isAtTargetVelocity = isAtTargetVelocity();

        if (isMoving && isAtTargetVelocity) {
            kickerServo.setPosition(KICKER_SERVO_RAISED_POSITION);
        } else {
            Log.i(LOG_TAG, "Not safe to raise kicker: moving: " + isMoving + ", at velocity: " + isAtTargetVelocity);
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