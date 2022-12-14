/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
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
 **/

package com.hfrobots.tnt.util;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * An OpMode that allows you to test any/all of the motors on a robot
 *
 * left stick (y) - set motor power (when not running to position)
 * right stick (y) - increment/decrement relative target position
 * left bumper - toggle float/brake power mode
 * right bumper - switch motors
 * a button - run to desired relative position using encoders
 */
@TeleOp(name="Velocity Tester", group="Utilities")
@Disabled
public class VelocityTester extends OpMode {
    private List<NamedDeviceMap.NamedDevice<DcMotorEx>> namedMotors;
    private Map<DcMotorEx, String> motorsToNames = new HashMap<>();
    private int currentListPosition;
    private double targetVelocity = 0.0D;

    private DebouncedButton bButton;

    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

    private Stopwatch stopwatch = Stopwatch.createUnstarted();

    @Override
    public void init() {
        NamedDeviceMap namedDeviceMap = new NamedDeviceMap(hardwareMap);
        namedMotors = namedDeviceMap.getAll(DcMotorEx.class);
        currentListPosition = 0;

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);

        bButton = new DebouncedButton(ninjaGamePad.getBButton());
        rightBumper = new DebouncedButton(ninjaGamePad.getRightBumper());
        dpadUp = new DebouncedButton(ninjaGamePad.getDpadUp());
        dpadDown = new DebouncedButton(ninjaGamePad.getDpadDown());
    }

    @Override
    public void loop() {
        if (namedMotors.isEmpty()) {
            telemetry.addData("No DC Motors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentListPosition++;

            if (currentListPosition == namedMotors.size()) {
                currentListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<DcMotorEx> namedDcMotor = namedMotors.get(currentListPosition);
        DcMotorEx currentMotor = namedDcMotor.getDevice();
        String motorName = namedDcMotor.getName();

        float leftStickYPosition = -gamepad1.left_stick_y;

        targetVelocity += leftStickYPosition;

        if (dpadUp.getRise()) {
            targetVelocity += 10;
        }

        if (dpadDown.getRise()) {
            targetVelocity -= 10;
        }

        if (bButton.getRise()) {
            targetVelocity = 0;
            currentMotor.setPower(0.0);
        } else {
            currentMotor.setPower(1.0);
        }

        currentMotor.setVelocity(targetVelocity);

        updateTelemetry(currentMotor, motorName, leftStickYPosition);
    }

    protected void updateTelemetry(DcMotorEx currentMotor, String motorName, float leftStickYPosition) {
        double currentVelocity = currentMotor.getVelocity();

        telemetry.addData("motor ",  "%s - %.2f - tV %.2f - cV %.2f",
                motorName,
                leftStickYPosition,
                targetVelocity,
                currentVelocity);
        updateTelemetry(telemetry);
    }
}