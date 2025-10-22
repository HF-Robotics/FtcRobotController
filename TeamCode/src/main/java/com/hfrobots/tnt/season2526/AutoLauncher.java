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

import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.season2324.Shared;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = AutoLauncher.OP_MODE_NAME, group = "util")
public class AutoLauncher extends OpMode {
    public static final String OP_MODE_NAME = "00 Auto launcher demo";

    private List<LynxModule> allHubs;

    private WheeledLauncher launcher;

    private Carousel carousel;

    private StateMachine launcherStateMachine;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            final Ticker ticker = Ticker.systemTicker();

            launcher = new WheeledLauncher(hardwareMap);
            carousel = new Carousel(hardwareMap);

            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            launcherStateMachine = new StateMachine(telemetry);

            // FIXME: Think about how to optimize the spin up time once this is
            //        wired into full auto
            final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

            State carouselHomeState = carousel.new HomeLocationState(telemetry, ticker);

            //sequenceOfStates.addSequential(carouselHomeState);

            addOneIndexAndLaunchIteration(sequenceOfStates, ticker);
            addOneIndexAndLaunchIteration(sequenceOfStates, ticker);
            addOneIndexAndLaunchIteration(sequenceOfStates, ticker);

            sequenceOfStates.addSequential(newDoneState("Done!"));

            launcherStateMachine.addSequence(sequenceOfStates);
        });
    }

    protected State newDoneState(String name) {
        return new State(name, telemetry) {
            private boolean issuedStop = false;

            @Override
            public State doStuffAndGetNextState() {
                // FIXME: Stop everything on the robot here
                if (!issuedStop) {
                    launcher.stopLauncher();
                    issuedStop = true;
                }

                return this;
            }

            @Override
            public void resetToStart() {
                issuedStop = false;
            }
        };
    }

    private void addOneIndexAndLaunchIteration(final SequenceOfStates sequenceOfStates, final Ticker ticker) {
        LauncherToSpeedState toSpeedState = new LauncherToSpeedState(telemetry);

        State carouselNextLaunchIndexState = carousel.new NextLaunchIndexState(telemetry, ticker);

        sequenceOfStates.addSequential(carouselNextLaunchIndexState);
        sequenceOfStates.addSequential(toSpeedState);
        sequenceOfStates.addRunnableStep("Raise kicker", () -> launcher.raiseKickerNoMatterWhat());
        sequenceOfStates.addWaitStep("Wait raise kicker", 1500, TimeUnit.MILLISECONDS);
        sequenceOfStates.addRunnableStep("Lower kicker", () -> launcher.lowerKicker());
        sequenceOfStates.addWaitStep("Wait lower kicker", 300, TimeUnit.MILLISECONDS);
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

            launcherStateMachine.doOneStateLoop();

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
