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

package com.hfrobots.tnt.season2526.small;

import com.ftc9929.corelib.control.RangeInput;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.season2526.WheeledLauncher;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.IterationListener;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SmallWheeledLauncher extends WheeledLauncher {

    @Override
    protected void setupHoodController(final HardwareMap hardwareMap,
                                       final Telemetry telemetry,
                                       final Ticker ticker) {
        // No hood on this robot!
    }

    public SmallWheeledLauncher(final HardwareMap hardwareMap,
                                final Telemetry telemetry,
                                final Ticker ticker) {
        super(hardwareMap, telemetry, ticker, null);
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void lowerKicker() {
        kickerServo.setPosition(KICKER_SERVO_RAISED_POSITION);
    }

    public void safelyRaiseKicker() {
        // if (isMoving() && isAtTargetVelocity()) {
        kickerServo.setPosition(0);
        // }
    }

    public void raiseKickerNoMatterWhat() {
        kickerServo.setPosition(0);
    }
}