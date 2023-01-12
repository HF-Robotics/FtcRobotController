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

package com.hfrobots.tnt.season2223;

import com.ftc9929.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RumbleTarget;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Builder;

public class ConeAndJunctionAlignment implements PeriodicTask {
    private final AlignmentIndicator alignmentIndicator;

    private final ConeLocalizer coneLocalizer;

    private final OnOffButton doConeAlignment;

    private final OnOffButton doJunctionAlignment;

    private final RumbleTarget driverRumble;

    private final RumbleTarget operatorRumble;

    @Builder
    private ConeAndJunctionAlignment(final HardwareMap hardwareMap,
                                     final OnOffButton doConeAlignment,
                                     final OnOffButton doJunctionAlignment,
                                     final RumbleTarget driverRumble,
                                     final RumbleTarget operatorRumble,
                                     final Telemetry telemetry) {
        this.driverRumble = driverRumble;
        this.operatorRumble = operatorRumble;
        this.doConeAlignment = doConeAlignment;
        this.doJunctionAlignment = doJunctionAlignment;

        alignmentIndicator = new AlignmentIndicator(hardwareMap, this.driverRumble, this.operatorRumble, telemetry);
        coneLocalizer = new ConeLocalizer(hardwareMap);
    }

    @Override
    public void periodicTask() {
        if (doConeAlignment != null && doConeAlignment.isPressed()) {
            doConeAlignment();
        } else if (doJunctionAlignment != null && doJunctionAlignment.isPressed()) {
            doJunctionAlignment();
        } else {
            alignmentIndicator.indicateInactive();
        }
    }

    private void doConeAlignment() {
        ConeLocalizer.ConeDistances coneDistances = coneLocalizer.getConeDistances();

        alignmentIndicator.distancesToIndicators(coneDistances);
    }

    private void doJunctionAlignment() {
        ConeLocalizer.ConeDistances coneDistances = coneLocalizer.getConeDistances();

        double distanceToJunctionMm = coneDistances.getCenterDistanceMm();

        if (coneDistances.isDistanceSensorsWorking() && distanceToJunctionMm < 75) {
            driverRumble.rumbleBlips(1);
            operatorRumble.rumbleBlips(1);
        }
    }
}