/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.util;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.RangeInput;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Motor Velocity Tester", group="Utilities")
public class MotorVelocityTester extends OpMode {

    private int lastFrontEncoderPos = Integer.MIN_VALUE;

    private double velocity;

    private List<NamedDeviceMap.NamedDevice<DcMotorEx>> namedMotors;
    private Map<DcMotor, String> motorsToNames = new HashMap<>();
    private int currentListPosition;

    private DebouncedButton aButton;

    private DebouncedButton bButton;

    private DebouncedButton rightBumper;

    private RangeInput leftStickY;

    private double encoderClicksPerSec;

    private double frontEncoderTicksPerSecond = 0;

    @Override
    public void init() {
        NamedDeviceMap namedDeviceMap = new NamedDeviceMap(hardwareMap);
        namedMotors = namedDeviceMap.getAll(DcMotorEx.class);
        currentListPosition = 0;

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        aButton = ninjaGamePad.getAButton().debounced();
        bButton = ninjaGamePad.getBButton().debounced();
        rightBumper = ninjaGamePad.getRightBumper().debounced();

        leftStickY = ninjaGamePad.getLeftStickY();
    }


    @Override
    public void loop() {
        if (namedMotors.isEmpty()) {
            telemetry.addData("No DC Motors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            NamedDeviceMap.NamedDevice<DcMotorEx> namedDcMotor = namedMotors.get(currentListPosition);
            DcMotorEx currentMotor = namedDcMotor.getDevice();
            currentMotor.setVelocity(0);
            velocity = 0;
            currentListPosition++;

            if (currentListPosition == namedMotors.size()) {
                currentListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<DcMotorEx> namedDcMotor = namedMotors.get(currentListPosition);
        DcMotorEx currentMotor = namedDcMotor.getDevice();
        String motorName = namedDcMotor.getName();

        float stickPos = leftStickY.getPosition();

        velocity += -stickPos * 4.0;

        currentMotor.setVelocity(velocity);
        encoderClicksPerSec = currentMotor.getVelocity();

        telemetry.addData("motor ",  "%s - vel_sp %s - cur_vel %s",
                motorName,
                Double.toString(velocity),
                Double.toString(encoderClicksPerSec));
        updateTelemetry(telemetry);
    }
}