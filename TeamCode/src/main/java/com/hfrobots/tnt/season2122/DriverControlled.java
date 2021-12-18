/*
 Copyright (c) 2021 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season2122;

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

import org.apache.commons.math3.analysis.function.Max;

import java.util.List;

@TeleOp(name = "00 Frt Frnzy TeleOp")
public class DriverControlled extends OpMode {

    private Drivebase drivebase;

    private DriverControls driverControls;

    private OperatorControls operatorControls;

    private StatsDMetricSampler legacyMetricsSampler;

    private RobotMetricsSampler newMetricsSampler;

    private FreightManipulator freightManipulator;

    private CarouselMechanism carouselMechanism;

    private boolean useLegacyMetricsSampler = true;

    private List<LynxModule> allHubs;

    private DriveTeamSignal driveTeamSignal;

    @Override
    public void init() {
        final Ticker ticker = Ticker.systemTicker();

        drivebase = new Drivebase(hardwareMap);
        freightManipulator = new FreightManipulator(hardwareMap, telemetry, ticker);
        carouselMechanism = new CarouselMechanism(hardwareMap);

        driveTeamSignal = new DriveTeamSignal(hardwareMap, ticker);

        NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

        driverControls = DriverControls.builder()
                .driversGamepad(driversGamepad)
                .kinematics(drivebase).build();

        NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

        MaxMotorPowerMagnitude maxMotorPowerMagnitude = MaxMotorPowerMagnitude.forDrivebase(hardwareMap);

        operatorControls = OperatorControls.builder().operatorGamepad(operatorGamepad)
                .freightManipulator(freightManipulator)
                .carouselMechanism(carouselMechanism)
                .maxMotorPowerMagnitude(maxMotorPowerMagnitude).build();

        setupMetricsSampler(driversGamepad, operatorGamepad);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void setupMetricsSampler(NinjaGamePad driversGamepad, NinjaGamePad operatorGamepad) {
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

        // Let operator work with wobble goal gripper during setup
        operatorControls.periodicTask();
    }

    private void clearHubsBulkCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void start() {
        super.start();

        driveTeamSignal.startMatch();
    }

    @Override
    public void loop() {
        clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

        driverControls.periodicTask();
        operatorControls.periodicTask();

        driveTeamSignal.periodicTask();

        if (useLegacyMetricsSampler) {
            if (legacyMetricsSampler != null) {
                legacyMetricsSampler.doSamples();
            }
        } else {
            if (newMetricsSampler != null) {
                newMetricsSampler.doSamples();
            }
        }

        telemetry.update();
    }
}
