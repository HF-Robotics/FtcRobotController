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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WheeledLauncher {

    public static final double ONE_FULL_REV = 751.8;
    public static final double ONE_THIRD_REV = ONE_FULL_REV / 3;
    public static final double ONE_SIXTH_REV = ONE_THIRD_REV / 2;
    // These constants are the tunables for our launcher.
    private static final double KICKER_SERVO_RAISED_POSITION = 1;

    private static final double KICKER_SERVO_LOWERED_POSITION = 0;

    private static final double FAR_LAUNCH_VELOCITY = 2200;

    private static final double MEDIUM_LAUNCH_VELOCITY = 2060;

    private static final double CLOSE_LAUNCH_VELOCITY = 1780;

    // These are the components of our launcher
    private final DcMotorEx launcherMotor;

    private final Servo kickerServo;

    private final DcMotorEx carouselMotor;

    private final DigitalChannel carouselHomeLimit;

    private double requestedVelocity = 0;

    public WheeledLauncher(final HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");

        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        kickerServo = hardwareMap.get(Servo.class, "kickerServo");

        carouselHomeLimit = hardwareMap.get(DigitalChannel.class, "carouselHome");
    }

    public void updateTelemetry(final Telemetry telemetry) {
        telemetry.addData("Launcher",  "req_vel %s - cur_vel %s",
                Double.toString(requestedVelocity),
                Double.toString(launcherMotor.getVelocity()));
    }

    private int currentIntakeIndex = 0;

    private int currentLaunchIndex = 0;

    public void indexCarouselForIntake() {
        currentIntakeIndex = currentIntakeIndex + 1;

        if (currentIntakeIndex > 2) {
            currentIntakeIndex = 0;
        }

        double oneFullRev = 751.8;
        double oneThirdRev = oneFullRev / 3;

        double targetPos = currentIntakeIndex * oneThirdRev;

        carouselMotor.setTargetPosition((int)(targetPos));
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(0.2);
    }

    private double[] launchPositions = {
            0 + ONE_SIXTH_REV,
            ONE_SIXTH_REV + ONE_THIRD_REV,
            ONE_SIXTH_REV + ONE_THIRD_REV + ONE_THIRD_REV
    };

    public void indexCarouselForLaunch() {
        currentLaunchIndex = currentLaunchIndex + 1;

        if (currentLaunchIndex > 2) {
            currentLaunchIndex = 0;
        }

        double targetPos = launchPositions[currentLaunchIndex];

        carouselMotor.setTargetPosition((int)(targetPos));
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(0.2);
    }

    public void adjustCarousel(final RangeInput carouselThrottle) {
        float carouselThrottlePosition = carouselThrottle.getPosition();

        if (carouselThrottlePosition != 0) {
            carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (carouselMotor.isBusy()) {
            return;
        }

        carouselThrottlePosition /= 9;

        if (carouselThrottlePosition < 0) {
            if (!carouselHomeLimit.getState()) {
                carouselMotor.setPower(0);

                return;
            }
        }

        carouselMotor.setPower(carouselThrottlePosition);
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