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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
public class ConeLocalizationTest extends OpMode {
    Rev2mDistanceSensor leftDistanceSensor;
    Rev2mDistanceSensor centerDistanceSensor;
    Rev2mDistanceSensor rightDistanceSensor;

    private AlignmentIndicator alignmentIndicator;

    private List<LynxModule> allHubs;

    private StatsDMetricSampler legacyMetricsSampler;

    @Override
    public void init() {
        legacyMetricsSampler = new StatsDMetricSampler(hardwareMap,
                new NinjaGamePad(gamepad1), new NinjaGamePad(gamepad2));

        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        centerDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "centerDistanceSensor");
        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");

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

        final double leftDistanceMm = leftDistanceSensor.getDistance(DistanceUnit.MM);
        final double centerDistanceMm = centerDistanceSensor.getDistance(DistanceUnit.MM);
        final double rightDistanceMm = rightDistanceSensor.getDistance(DistanceUnit.MM);

        alignmentIndicator.distancesToLights(
                Range.clip(leftDistanceMm, 0, 300),
                Range.clip(centerDistanceMm, 0, 300),
                Range.clip(rightDistanceMm, 0, 300));

        if (legacyMetricsSampler != null) {
            legacyMetricsSampler.doSamples();
        }

        long endMs = System.currentTimeMillis();

        telemetry.addData("Left (mm): ", leftDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Center (mm): ", centerDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Right (mm): ", rightDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Cycle (ms): ", endMs - startMs);
        updateTelemetry(telemetry);
    }

    private void clearHubsBulkCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}