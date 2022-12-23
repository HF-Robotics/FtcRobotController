/*
 Copyright (c) 2022 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2223;

import com.hfrobots.tnt.corelib.sensors.RevLedIndicator;
import com.qualcomm.robotcore.hardware.HardwareMap;

class AlignmentIndicator {
    private final RevLedIndicator leftLed;

    private final RevLedIndicator centerLed;

    private final RevLedIndicator rightLed;

    public AlignmentIndicator(final HardwareMap hardwareMap) {
        leftLed = new RevLedIndicator(hardwareMap, "leftConeLed");
        centerLed = new RevLedIndicator(hardwareMap, "centerConeLed");
        rightLed = new RevLedIndicator(hardwareMap, "rightConeLed");
    }

    public void distancesToLights(final double leftDistanceMm,
                                  final double centerDistanceMm,
                                  final double rightDistanceMm) {
        doIndicator(leftDistanceMm, leftLed);
        doIndicator(centerDistanceMm, centerLed);
        doIndicator(rightDistanceMm, rightLed);
    }

    private void doIndicator(final double distanceMm,
                             final RevLedIndicator indicator) {
        if (distanceMm < 50) {
            indicator.setColor(RevLedIndicator.LedColor.GREEN);
        } else if (distanceMm < 100) {
            indicator.setColor(RevLedIndicator.LedColor.AMBER);
        } else if (distanceMm < 300) {
            indicator.setColor(RevLedIndicator.LedColor.RED);
        } else {
            indicator.setColor(RevLedIndicator.LedColor.OFF);
        }
    }
}
