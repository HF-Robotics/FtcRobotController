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

import com.ftc9929.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class ConeLocalizationTest extends OpMode {

    private AlignmentIndicator alignmentIndicator;

    private ConeLocalizer coneLocalizer;

    private List<LynxModule> allHubs;

    private StatsDMetricSampler legacyMetricsSampler;

    @Override
    public void init() {
        legacyMetricsSampler = new StatsDMetricSampler(hardwareMap,
                new NinjaGamePad(gamepad1), new NinjaGamePad(gamepad2));

        coneLocalizer = new ConeLocalizer(hardwareMap);
        alignmentIndicator = new AlignmentIndicator(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        long startMs = System.currentTimeMillis();

        clearHubsBulkCaches();

        ConeLocalizer.ConeDistances coneDistances = coneLocalizer.getConeDistances();

        alignmentIndicator.distancesToLights(coneDistances);

        if (legacyMetricsSampler != null) {
            legacyMetricsSampler.doSamples();
        }

        long endMs = System.currentTimeMillis();

        telemetry.addData("Left (mm): ", coneDistances.getLeftDistanceMm());
        telemetry.addData("Center (mm): ", coneDistances.getCenterDistanceMm());
        telemetry.addData("Right (mm): ", coneDistances.getRightDistanceMm());

        telemetry.addData("Cycle (ms): ", endMs - startMs);
        updateTelemetry(telemetry);
    }

    private void clearHubsBulkCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

}