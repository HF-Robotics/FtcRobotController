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

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.controllers.LinearLiftController;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringMechanism implements PeriodicTask {
    private LinearLiftController liftController;

    public ScoringMechanism(final HardwareMap hardwareMap) {
        // Need a motor
        final ExtendedDcMotor liftMotor =
                NinjaMotor.asNeverest20(
                        hardwareMap.get(DcMotorEx.class, "liftMotor"));

        // need lower limit (if available)

        // need upper limit switch (if available)

        liftController = LinearLiftController.builder().liftMotor(liftMotor).build();
    }

    public void setLiftThrottle(final RangeInput throttle) {
        liftController.setLiftThrottle(throttle);
    }

    public void setEmergencyStop(final DebouncedButton emergencyStopButton) {
        liftController.setLiftEmergencyStopButton(emergencyStopButton);
    }

    public void setUnsafe(final OnOffButton unsafe) {
        liftController.setLimitOverrideButton(unsafe);
    }

    @Override
    public void periodicTask() {
        liftController.periodicTask();
    }
}
