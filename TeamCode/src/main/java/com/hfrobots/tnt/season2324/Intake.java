/*
 Copyright (c) 2023 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2324;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Intake {
    private DcMotorEx leftIntakeMotor;

    private DcMotorEx rightIntakeMotor;

    private final CRServo intakeServo1;

    private final CRServo intakeServo2;

    public Intake(final HardwareMap hardwareMap) {
        leftIntakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");
        rightIntakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");

        // GoBilda 6000 RPM motor rotates CCW when viewed from above
        //
        // Left intake wheel needs to rotate CW when viewed from above
        //
        // Right intake wheel needs to rotate CCW when viewed from above
        leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
    }

    public void in(double speed) {
        speed = Math.abs(speed);
        speed = Range.clip(speed, 0, 1);
        leftIntakeMotor.setPower(speed);
        rightIntakeMotor.setPower(speed);
        intakeServo1.setPower(1);
        intakeServo2.setPower(1);
    }

    public void stop() {
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
        intakeServo1.setPower(0);
        intakeServo2.setPower(0);
    }

    public void out(double speed) {
        speed = Math.abs(speed);
        speed = Range.clip(speed, 0, 1);
        leftIntakeMotor.setPower(-speed);
        rightIntakeMotor.setPower(-speed);
        intakeServo1.setPower(-1);
        intakeServo2.setPower(-1);
    }
}
