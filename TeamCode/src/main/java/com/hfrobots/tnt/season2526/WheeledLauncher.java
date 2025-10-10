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

import com.ftc9929.corelib.control.RangeInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WheeledLauncher {

    // These constants are the tunables for our launcher.
    private static final double KICKER_SERVO_RAISED_POSITION = 1;

    private static final double KICKER_SERVO_LOWERED_POSITION = 0;

    private static final double FAR_LAUNCH_VELOCITY = 2200;

    private static final double MEDIUM_LAUNCH_VELOCITY = 1880;

    private static final double CLOSE_LAUNCH_VELOCITY = 1780;

    // These are the components of our launcher
    private final DcMotorEx launcherMotor;

    private final Servo kickerServo;

    private final DcMotorEx carouselMotor;

    private double requestedVelocity = 0;

    public WheeledLauncher(final HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");

        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");

        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
    }

    public void updateTelemetry(final Telemetry telemetry) {
        telemetry.addData("Launcher",  "req_vel %s - cur_vel %s",
                Double.toString(requestedVelocity),
                Double.toString(launcherMotor.getVelocity()));
    }

    public void adjustCarousel(final RangeInput carouselThrottle) {
        carouselMotor.setPower(carouselThrottle.getPosition());
    }

    public void adjustVelocity(final RangeInput adjustmentThrottle) {
        double adjustmentAmount = -adjustmentThrottle.getPosition();

        if (adjustmentAmount != 0) {
            maybeSetLaunchVelocity(requestedVelocity + (adjustmentAmount * 4.0));
        }
    }

    public void closeLaunchVelocity() {
        maybeSetLaunchVelocity(CLOSE_LAUNCH_VELOCITY);
    }

    public void mediumLaunchVelocity() {
        maybeSetLaunchVelocity(MEDIUM_LAUNCH_VELOCITY);
    }

    public void farLaunchVelocity() {
        maybeSetLaunchVelocity(FAR_LAUNCH_VELOCITY);
    }

    public void stopLauncher() {
        maybeSetLaunchVelocity(0);
    }

    public boolean isMoving() {
        return launcherMotor.getVelocity() < 50;
    }

    public boolean isAtTargetVelocity() {
        double currentVelocity = launcherMotor.getVelocity();

        return Math.abs(requestedVelocity - currentVelocity) < 200;
    }

    public void lowerKicker() {
        kickerServo.setPosition(KICKER_SERVO_LOWERED_POSITION);
    }

    public void safelyRaiseKicker() {
        // if (isMoving() && isAtTargetVelocity()) {
            kickerServo.setPosition(KICKER_SERVO_RAISED_POSITION);
       // }
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
}