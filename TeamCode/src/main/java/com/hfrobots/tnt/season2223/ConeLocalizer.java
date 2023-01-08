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

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import lombok.Builder;
import lombok.Value;

class ConeLocalizer {
    @Builder
    @Value
    static class ConeDistances {
        final double leftDistanceMm;
        final double centerDistanceMm;
        final double rightDistanceMm;

        final boolean distanceSensorsWorking;
    }

    private Rev2mDistanceSensor leftConeDistanceSensor;
    private Rev2mDistanceSensor centerConeDistanceSensor;
    private Rev2mDistanceSensor rightConeDistanceSensor;

    public ConeLocalizer(final HardwareMap hardwareMap) {
        try {
            leftConeDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftConeDistanceSensor");
        } catch (Exception ex) {
            leftConeDistanceSensor = null;
            Log.e(LOG_TAG, "Left cone distance sensor failure", ex);
        }

        try {
            centerConeDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "centerConeDistanceSensor");
        } catch (Exception ex) {
            centerConeDistanceSensor = null;
            Log.e(LOG_TAG, "Center cone distance sensor failure", ex);
        }

        try {
            rightConeDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightConeDistanceSensor");
        } catch (Exception ex) {
            rightConeDistanceSensor = null;
            Log.e(LOG_TAG, "Right cone distance sensor failure", ex);
        }
    }

    public ConeDistances getConeDistances() {
        if (leftConeDistanceSensor == null || centerConeDistanceSensor == null || rightConeDistanceSensor == null) {
            return brokenSensors();
        }

        try {
            final double leftDistanceMm = leftConeDistanceSensor.getDistance(DistanceUnit.MM);
            final double centerDistanceMm = centerConeDistanceSensor.getDistance(DistanceUnit.MM);
            final double rightDistanceMm = rightConeDistanceSensor.getDistance(DistanceUnit.MM);

            ConeDistances coneDistances = ConeDistances.builder()
                    .leftDistanceMm(Range.clip(leftDistanceMm, 0, 300))
                    .centerDistanceMm(Range.clip(centerDistanceMm, 0, 300))
                    .rightDistanceMm(Range.clip(rightDistanceMm, 0, 300))
                    .distanceSensorsWorking(true).build();

            return coneDistances;
        } catch (Exception ex) {
            Log.e(LOG_TAG, "Cone distance sensor failure", ex);

            return brokenSensors();
        }
    }

    private ConeDistances brokenSensors() {
        return ConeDistances.builder()
                .leftDistanceMm(0)
                .centerDistanceMm(0)
                .rightDistanceMm(0)
                .distanceSensorsWorking(false).build();
    }
}
