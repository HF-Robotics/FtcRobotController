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

package com.hfrobots.tnt.season2324;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.metrics.RobotMetricsSampler;
import com.ftc9929.metrics.StatsdMetricsReporter;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = CenterstageDriverControlled.CENTERSTAGE_TELE_OP_NAME)
public class CenterstageDriverControlled extends OpMode {

    public static final String CENTERSTAGE_TELE_OP_NAME = "CS TeleOp";

    private CenterstageDrivebase drivebase;

    private CenterstageDriverControls driverControls;

    private CenterstageOperatorControls operatorControls;

    private StatsDMetricSampler legacyMetricsSampler;

    private RobotMetricsSampler newMetricsSampler;

    private boolean emitMetrics = false;

    private final boolean useLegacyMetricsSampler = true;

    private List<LynxModule> allHubs;

    private Intake intake;

    private Hanger hanger;

    private ScoringMechanism scoringMechanism;

    private DroneLauncher droneLauncher;

    private CenterstageDriveTeamSignal driveTeamSignal;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            final Ticker ticker = Ticker.systemTicker();

            drivebase = new CenterstageDrivebase(hardwareMap);

            NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

            driverControls = CenterstageDriverControls.builder()
                    .driversGamepad(driversGamepad)
                    .kinematics(drivebase).build();

            NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

            intake = new Intake(hardwareMap);

            hanger = new Hanger(hardwareMap);

            scoringMechanism = ScoringMechanism.builderFromHardwareMap(hardwareMap, telemetry).build();

            driveTeamSignal = new CenterstageDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

            droneLauncher = DroneLauncher.builder().driveTeamSignal(driveTeamSignal)
                    .hardwareMap(hardwareMap).build();
            droneLauncher.setSafetyOn(false);

            operatorControls = CenterstageOperatorControls.builder().operatorGamepad(operatorGamepad)
                    .intake(intake)
                    .hanger(hanger)
                    .scoringMechanism(scoringMechanism)
                    .droneLauncher(droneLauncher)
                    .driveTeamSignal(driveTeamSignal)
                    .build();

            maybeSetupMetricsSampler(driversGamepad, operatorGamepad);

            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        });
    }

    private void maybeSetupMetricsSampler(NinjaGamePad driversGamepad, NinjaGamePad operatorGamepad) {
        if (!emitMetrics) {
            return;
        }

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

    @Override
    public void init_loop() {
        clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!
        operatorControls.periodicTask();
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
            scoringMechanism.setStarted(true);
            droneLauncher.setSafetyOn(true);
            driveTeamSignal.startMatch();
        });
    }

    @Override
    public void loop() {
        Shared.withBetterErrorHandling(() -> {
            clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

            driverControls.periodicTask();
            operatorControls.periodicTask();

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

            driveTeamSignal.periodicTask();

            telemetry.update();
        });
    }
}
