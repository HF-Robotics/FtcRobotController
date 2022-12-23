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
import com.ftc9929.corelib.control.OnOffButton;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * An OpMode that allows you to test any/all of the motors on a robot
 *
 * left stick (y) - set motor power (when not running to position)
 * right stick (y) - increment/decrement relative target position
 * left bumper - toggle float/brake power mode
 * right bumper - switch motors
 * a button - run to desired relative position using encoders
 */
@TeleOp(name="CRServo Tester", group="Utilities")
public class CRServoTester extends OpMode {
    private List<NamedDeviceMap.NamedDevice<CRServo>> namedServos;
    private Map<CRServo, String> servosToNames = new HashMap<>();
    private int currentListPosition;
    private int desiredPosition;

    private OnOffButton aButton;

    private OnOffButton bButton;

    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

    private double encoderClicksPerSec;

    private int lastEncoderCount = 0;

    private Stopwatch stopwatch = Stopwatch.createUnstarted();

    @Override
    public void init() {
        NamedDeviceMap namedDeviceMap = new NamedDeviceMap(hardwareMap);
        namedServos = namedDeviceMap.getAll(CRServo.class);
        currentListPosition = 0;

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        aButton = ninjaGamePad.getAButton();
        bButton = ninjaGamePad.getBButton();
        rightBumper = new DebouncedButton(ninjaGamePad.getRightBumper());
        dpadUp = new DebouncedButton(ninjaGamePad.getDpadUp());
        dpadDown = new DebouncedButton(ninjaGamePad.getDpadDown());
    }

    @Override
    public void loop() {
        if (namedServos.isEmpty()) {
            telemetry.addData("No DC Motors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentListPosition++;

            if (currentListPosition == namedServos.size()) {
                currentListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<CRServo> namedDcServo = namedServos.get(currentListPosition);
        CRServo currentServo = namedDcServo.getDevice();
        String servoName = namedDcServo.getName();

        if (bButton.isPressed()) {
            currentServo.setPower(1);
        } else if (aButton.isPressed()) {
            currentServo.setPower(-1);
        } else {
            float leftStickYPosition = -gamepad1.left_stick_y;
            if (leftStickYPosition != 0) {
                currentServo.setPower(leftStickYPosition);
            }
        }

        updateTelemetry(currentServo, servoName);
    }

    protected void updateTelemetry(CRServo currentServo, String servoName) {

        telemetry.addData("crServo ",  "%s - pow %f", servoName, currentServo.getPower());
        updateTelemetry(telemetry);
    }
}
