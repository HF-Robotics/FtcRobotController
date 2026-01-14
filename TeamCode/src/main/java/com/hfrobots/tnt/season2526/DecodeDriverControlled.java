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

package com.hfrobots.tnt.season2526;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.metrics.RobotMetricsSampler;
import com.ftc9929.metrics.StatsdMetricsReporter;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.hfrobots.tnt.season2324.Shared;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = DecodeDriverControlled.OP_MODE_NAME)
public class DecodeDriverControlled extends OpMode {
    public static final String OP_MODE_NAME = "00 DECODE";

    private final boolean emitMetrics = false;

    private DecodeDrivebase drivebase;

    private DecodeDriverControls driverControls;

    private DecodeOperatorControls operatorControls;

    private StatsDMetricSampler legacyMetricsSampler;

    private RobotMetricsSampler newMetricsSampler;;

    private final boolean useLegacyMetricsSampler = true;

    private List<LynxModule> allHubs;

    private WheeledLauncher launcher;

    private GenevaCarousel carousel;

    private DecodeDriveTeamSignal driveTeamSignal;

    private AprilTagAligner aprilTagAligner;
    
    boolean canUseAprilTags = false;
    private RollerIntake intake;

    private Kickstand kickstand;

    private Ticker ticker;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = Ticker.systemTicker();

            drivebase = new DecodeDrivebase(hardwareMap);

            setupMechanisms();

            NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

            driverControls = DecodeDriverControls.builder()
                    .driversGamepad(driversGamepad)
                    .launcher(launcher)
                    .kinematics(drivebase).build();
            
            final NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

            operatorControls = DecodeOperatorControls.builder()
                    .operatorGamepad(operatorGamepad)
                    .intake(intake)
                    .carousel(carousel)
                    .launcher(launcher)
                    .kickstand(kickstand).build();

            setupMetricsSampler(driversGamepad, operatorGamepad);

            try {
                driveTeamSignal = new DecodeDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);
            } catch (IllegalArgumentException ex) {
                driveTeamSignal = null;
            }

            try {
                aprilTagAligner = new AprilTagAligner(telemetry, drivebase, hardwareMap, ticker);
                canUseAprilTags = true;
            } catch (Exception ex) {
                Log.e(LOG_TAG, "Unable to initialize AprilTags", ex);
                canUseAprilTags = false;
            }

            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        });
    }

    private void setupMechanisms() {
        try {
            launcher = new WheeledLauncher(hardwareMap, telemetry, ticker);
        } catch (IllegalArgumentException ex) {
            launcher = null;
        }

        try {
            intake = new RollerIntake(hardwareMap);
        } catch (IllegalArgumentException ex) {
            intake = null;
        }

        try {
            carousel = new OGGenevaCarousel(hardwareMap, telemetry);
        } catch (IllegalArgumentException ex) {
            carousel = null;
        }

        try {
            kickstand = new Kickstand(hardwareMap);
        } catch (IllegalArgumentException ex) {
            kickstand = null;
        }
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

            if (driveTeamSignal != null) {
                driveTeamSignal.startMatch();
            }

            launcher.homeHood();
        });
    }

    @Override
    public void loop() {
        Shared.withBetterErrorHandling(() -> {
            clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

            if (canUseAprilTags) {
                aprilTagAligner.periodicTask();
            }
            
            if (canUseAprilTags && driverControls.rightBumper.isPressed()) {
                aprilTagAligner.aimToDetectedAprilTag();
            } else {
                driverControls.periodicTask();
            }
            
            operatorControls.periodicTask();

            if (driveTeamSignal != null) {
                driveTeamSignal.periodicTask();
            }

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

            telemetry.update();
        });
    }
}
