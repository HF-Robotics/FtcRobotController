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

package com.hfrobots.tnt.season2627.opmodes;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.metrics.RobotMetricsSampler;
import com.ftc9929.metrics.StatsdMetricsReporter;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.control.RumbleTarget;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.hfrobots.tnt.season2324.Shared;
import com.hfrobots.tnt.season2526.drivebase.DecodeDrivebase;
import com.hfrobots.tnt.season2526.driveteam.DecodeDriveTeamSignal;
import com.hfrobots.tnt.season2526.driveteam.DecodeDriverControls;
import com.hfrobots.tnt.season2526.driveteam.DecodeOperatorControls;
import com.hfrobots.tnt.season2526.driveteam.GamepadLed;
import com.hfrobots.tnt.season2526.mechanisms.AbsGenevaCarousel;
import com.hfrobots.tnt.season2526.mechanisms.ArtifactDetector;
import com.hfrobots.tnt.season2526.mechanisms.GenevaCarousel;
import com.hfrobots.tnt.season2526.mechanisms.Kickstand;
import com.hfrobots.tnt.season2526.mechanisms.RollerIntake;
import com.hfrobots.tnt.season2526.mechanisms.WheeledLauncher;
import com.hfrobots.tnt.season2526.vision.AprilTagAligner;
import com.hfrobots.tnt.season2627.BiobuzzDrivebase;
import com.hfrobots.tnt.season2627.driveteam.BiobuzzDriverControls;
import com.hfrobots.tnt.season2627.driveteam.BiobuzzOperatorControls;
import com.hfrobots.tnt.util.TimeTracker;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = BiobuzzDriverControlled.OP_MODE_NAME)
public class BiobuzzDriverControlled extends OpMode {
    public static final String OP_MODE_NAME = "00 BIOBUZZ";

    private boolean emitMetrics = false;

    private BiobuzzDrivebase drivebase;

    private BiobuzzDriverControls driverControls;

    private BiobuzzOperatorControls operatorControls;

    private StatsDMetricSampler legacyMetricsSampler;

    private RobotMetricsSampler newMetricsSampler;;

    private final boolean useLegacyMetricsSampler = true;

    private List<LynxModule> allHubs;

    private Ticker ticker;
    private NinjaGamePad driversGamepad;
    private NinjaGamePad operatorGamepad;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = Ticker.systemTicker();

            drivebase = new BiobuzzDrivebase(hardwareMap);


            setupMechanisms();

            driversGamepad = new NinjaGamePad(gamepad1);

            driverControls = BiobuzzDriverControls.builder()
                    .driversGamepad(driversGamepad)
                    .kinematics(drivebase)
                    .build();

            operatorGamepad = new NinjaGamePad(gamepad2);

            operatorControls = BiobuzzOperatorControls.builder()
                    .operatorGamepad(operatorGamepad).build();

            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        });
    }

    private void setupMechanisms() {

    }

    private void setupMetricsSampler(NinjaGamePad driversGamepad, NinjaGamePad operatorGamepad) {
        if (emitMetrics) {
            try {
                if (useLegacyMetricsSampler) {
                    legacyMetricsSampler = new StatsDMetricSampler(hardwareMap, driversGamepad, operatorGamepad);
                } else {
                    StatsdMetricsReporter metricsReporter = StatsdMetricsReporter.builder()
                            .metricsServerHost("192.168.43.78").
                            metricsServerPortNumber(8126).build();

                    newMetricsSampler = RobotMetricsSampler.builder()
                            .metricsReporter(metricsReporter)
                            .hardwareMap(hardwareMap)
                            .driverControls(driversGamepad)
                            .operatorControls(operatorGamepad).build();
                }
            } catch (Exception ex) {
                Log.w(LOG_TAG, "Unable to setup metrics sampler", ex);
            }
        }
    }

    @Override
    public void init_loop() {
        clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

        /*
        if (driveTeamSignal.getChosenAlliance() == null) {
            if (gamepad1.bWasPressed()) {
                driveTeamSignal.setAlliance(Constants.Alliance.RED);
            } else if (gamepad1.xWasPressed()) {
                driveTeamSignal.setAlliance(Constants.Alliance.BLUE);
            }
        }*/

        if (gamepad1.circleWasPressed()) {
            emitMetrics = !emitMetrics;

            if (emitMetrics) {
                setupMetricsSampler(driversGamepad, operatorGamepad);
            }
        }

        telemetry.addLine("Emit metrics: " + emitMetrics + " (press circle to toggle)");
        telemetry.update();
    }

    private void clearHubsBulkCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void start() {
        Shared.withBetterErrorHandling(() -> {
            super.start();

            /*
            if (driveTeamSignal != null) {
                driveTeamSignal.startMatch();
            }*/
        });
    }

    private final TimeTracker timer = new TimeTracker(telemetry, "loop");

    @Override
    public void loop() {
        Shared.withBetterErrorHandling(() -> {
            timer.trackTime(() -> {
                clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!
                driverControls.periodicTask();
                operatorControls.periodicTask();

                // if (driveTeamSignal != null) {
                //    driveTeamSignal.periodicTask();
                // }

                if (emitMetrics) {
                    if (useLegacyMetricsSampler) {
                        if (legacyMetricsSampler != null) {
                            legacyMetricsSampler.doSamples();
                        }
                    } else {
                        if (newMetricsSampler != null) {
                            newMetricsSampler.doSamples();
                        }
                    }
                }


            });
        });

        telemetry.update();
    }
}
