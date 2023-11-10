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

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import lombok.Builder;
import lombok.Setter;

public class DroneLauncher implements PeriodicTask {
    private final static double TRIGGER_SERVO_CLOSED_POS = 0;

    private final static double TRIGGER_SERVO_OPEN_POS = 0.73;

    private final Servo triggerServo;

    @Setter
    private OnOffButton triggerButton;

    @Setter
    private OnOffButton unsafeButton;

    @Setter
    private boolean safetyOn = false;

    private CenterstageDriveTeamSignal driveTeamSignal;

    @Builder
    public DroneLauncher(final HardwareMap hardwareMap,
                         final CenterstageDriveTeamSignal driveTeamSignal) {
        this.driveTeamSignal = driveTeamSignal;
        this.triggerServo = hardwareMap.get(Servo.class, "launcherTriggerServo");
        this.triggerServo.setPosition(TRIGGER_SERVO_CLOSED_POS);
    }

    @Override
    public void periodicTask() {
        final boolean canOpenTrigger;

        if (!safetyOn) {
            canOpenTrigger = true;
        } else if (driveTeamSignal != null && driveTeamSignal.isEndGame()) {
            canOpenTrigger = true;
        } else if (unsafeButton != null && unsafeButton.isPressed()) {
            canOpenTrigger = true;
        } else {
            canOpenTrigger = false;
        }

        if (canOpenTrigger) {
            if (triggerButton != null && triggerButton.isPressed()) {
                Log.d(LOG_TAG, "Opening trigger");
                triggerServo.setPosition(TRIGGER_SERVO_OPEN_POS);

                return;
            }
        }

        triggerServo.setPosition(TRIGGER_SERVO_CLOSED_POS);
    }
}
