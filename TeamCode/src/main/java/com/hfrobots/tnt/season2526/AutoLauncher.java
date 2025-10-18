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
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.metrics.RobotMetricsSampler;
import com.ftc9929.metrics.StatsdMetricsReporter;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.season2324.Shared;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

import lombok.NonNull;

@TeleOp(name = AutoLauncher.OP_MODE_NAME, group = "util")
public class AutoLauncher extends OpMode {
    public static final String OP_MODE_NAME = "00 Auto launcher demo";

    private final boolean emitMetrics = false;

    private DecodeDrivebase drivebase;

    private DecodeDriverControls driverControls;

    private DecodeOperatorControls operatorControls;

    private StatsDMetricSampler legacyMetricsSampler;

    private RobotMetricsSampler newMetricsSampler;;

    private final boolean useLegacyMetricsSampler = true;

    private List<LynxModule> allHubs;

    private WheeledLauncher launcher;

    private StateMachine launcherStateMachine;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            final Ticker ticker = Ticker.systemTicker();

            drivebase = new DecodeDrivebase(hardwareMap);

            NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

            driverControls = DecodeDriverControls.builder()
                    .driversGamepad(driversGamepad)
                    .kinematics(drivebase).build();

            RollerIntake intake;

            try {
                intake = new RollerIntake(hardwareMap);
            } catch (IllegalArgumentException ex) {
                intake = null;
            }

            try {
                launcher = new WheeledLauncher(hardwareMap);
            } catch (IllegalArgumentException ex) {
                launcher = null;
            }

            final NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

            operatorControls = DecodeOperatorControls.builder()
                    .operatorGamepad(operatorGamepad)
                    .intake(intake)
                    .launcher(launcher).build();

            setupMetricsSampler(driversGamepad, operatorGamepad);

            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            launcherStateMachine = new StateMachine(telemetry);

            LauncherToSpeedState toSpeedState = new LauncherToSpeedState(telemetry);
            State raiseKickerState = new RunnableState("Raise kicker", telemetry,
                    () -> {
                launcher.raiseKickerNoMatterWhat();
                    });
            State waitForKickerToRaiseState = new DelayState("wait a bit", telemetry, 1500, TimeUnit.MILLISECONDS);

            State lowerKickerState = new RunnableState("Lower kicker", telemetry,
                    () -> {
                        launcher.lowerKicker();
                    });

            State waitForKickerToLowerState = new DelayState("wait a bit", telemetry, 1500, TimeUnit.MILLISECONDS);

            toSpeedState.setNextState(raiseKickerState);
            raiseKickerState.setNextState(waitForKickerToRaiseState);
            waitForKickerToRaiseState.setNextState(lowerKickerState);
            lowerKickerState.setNextState(waitForKickerToLowerState);
            waitForKickerToLowerState.setNextState(toSpeedState);
            launcherStateMachine.setFirstState(toSpeedState);
        });
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
        });
    }

    @Override
    public void loop() {
        Shared.withBetterErrorHandling(() -> {
            clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

            driverControls.periodicTask();
            //operatorControls.periodicTask();
            launcherStateMachine.doOneStateLoop();

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

            launcher.updateTelemetry(telemetry);
            telemetry.update();
        });
    }

    class LauncherToSpeedState extends State {

        protected LauncherToSpeedState(Telemetry telemetry) {
            super("Speeding up", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            launcher.farLaunchVelocity();

            if (launcher.isAtTargetVelocity()) {
                return nextState;
            }

            return this;
        }

        @Override
        public void resetToStart() {

        }
    }
}
