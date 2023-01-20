/**
 Copyright (c) 2023 HF Robotics (http://www.hfrobots.com)
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * An OpMode that allows you to test any/all of the encoders on a robot
 * left bumper - toggle float/brake power mode
 * right bumper - switch motors
 */
@TeleOp(name="Encoder Tester", group="Utilities")
public class EncoderTester extends OpMode {
    private List<NamedDeviceMap.NamedDevice<DcMotor>> namedMotors;
    private Map<DcMotor, String> motorsToNames = new HashMap<>();
    private int currentListPosition;

    private DebouncedButton rightBumper;

    private int lastEncoderCount = 0;

    private Stopwatch stopwatch = Stopwatch.createUnstarted();

    @Override
    public void init() {
        NamedDeviceMap namedDeviceMap = new NamedDeviceMap(hardwareMap);
        namedMotors = namedDeviceMap.getAll(DcMotor.class);
        currentListPosition = 0;

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        rightBumper = new DebouncedButton(ninjaGamePad.getRightBumper());
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

        NamedDeviceMap.NamedDevice<DcMotor> namedDcMotor = namedMotors.get(currentListPosition);
        DcMotor currentMotor = namedDcMotor.getDevice();
        String motorName = namedDcMotor.getName();

        updateTelemetry(currentMotor, motorName);
    }

    protected void updateTelemetry(DcMotor currentMotor, String motorName) {
        int currentPosition = currentMotor.getCurrentPosition();

        telemetry.addData("motor ",  "%s - curpos %s",
                motorName,
                Integer.toString(currentPosition));

        updateTelemetry(telemetry);
    }
}
