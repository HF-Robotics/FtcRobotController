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

package com.hfrobots.tnt.season2425;

import static com.ftc9929.corelib.Constants.LOG_TAG;
import static com.hfrobots.tnt.season2425.IntoTheDeepDriverControlled.ITDEEP_TELE_OP;

import android.util.Log;
import android.util.Size;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Optional;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.google.common.collect.Sets;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.drive.mecanum.MultipleTrajectoriesFollowerState;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveBase;
import com.hfrobots.tnt.corelib.drive.mecanum.TurnState;
import com.hfrobots.tnt.corelib.state.ReadyCheckable;
import com.hfrobots.tnt.season2324.Shared;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Set;
import java.util.concurrent.TimeUnit;

import lombok.NonNull;

@TeleOp(name = "ITDEEP Demo", group="Utilities")
public class IntoTheDeepDemo extends OpMode {
    private Ticker ticker;


    // FIXME: The tasks our robot knows how to do - rename these to
    //  something meaningful for the season!
    private enum Task {
        HANG_SPECIMEN("Hang specimen"),
        HANG_SPECIMEN_NO_PARK("Hang specimen - no park"),
        HANG_SPECIMEN_NET_SIDE("Hang specimen - net side");
        // WORKOUT_TIME("Exercise the specimen lift");

        final String description;

        Task(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    private int selectedTaskIndex = 0;

    private Task[] possibleTaskChoices = Task.values();

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Constants.Alliance currentAlliance = Constants.Alliance.RED;

    private int initialDelaySeconds = 0;

    private IntoTheDeepOperatorControls operatorControls;

    private IntoTheDeepDriverControls driverControls;

    private IntoTheDeepDriveTeamSignal driveTeamSignal;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();
            setupDriverControls();
            setupOperatorControls();

            driveTeamSignal = new IntoTheDeepDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);
        });
    }

    private boolean configLocked = false;

    private Stopwatch unstallArmTimer = Stopwatch.createUnstarted();

    @Override
    public void init_loop() {
        if (unstallArmTimer.isRunning() && unstallArmTimer.elapsed(TimeUnit.SECONDS) > 30) {
            unstallArmTimer.stop();
            Log.d(LOG_TAG, "Un-stalling arm - waited too long in init()");
        }

        doAutoConfig();
        updateTelemetry(telemetry);
    }

    private Ticker createAndroidTicker() {
        return new Ticker() {
            public long read() {
                return android.os.SystemClock.elapsedRealtimeNanos();
            }
        };
    }

    private void doAutoConfig() {
        Shared.withBetterErrorHandling(() -> {
            if (driverControls == null) { // safety, need to double check whether we actually need this
                // not ready yet init() hasn't been called
                return;
            }

            driverControls.periodicTask();

            if (configLocked) {
                telemetry.addData("00", "LOCKED: Press Rt stick unlock");
            } else {
                telemetry.addData("00", "UNLOCKED: Press Lt stick lock");
            }

            telemetry.addData("01", "Alliance: %s", currentAlliance);
            telemetry.addData("02", "Task: %s", possibleTaskChoices[selectedTaskIndex].getDescription());
            telemetry.addData("03", "Delay %d sec", initialDelaySeconds);

            driveTeamSignal.setAlliance(currentAlliance);
            driveTeamSignal.periodicTask();
        });
    }

    private boolean stateMachineSetup = false;

    @Override
    public void loop() {

    }

    private void setupDriverControls() {
        driverControls = IntoTheDeepDriverControls.builder()
                .driversGamepad(new NinjaGamePad(gamepad1)).autoConfigTask(new IntoTheDeepDriverControls.InitLoopConfigTask() {
                    @Override
                    public void lockConfig() {
                        configLocked = true;
                    }

                    @Override
                    public void unlockConfig() {
                        configLocked = false;
                    }

                    @Override
                    public void chooseBlueAlliance() {
                        if (!configLocked) {
                            currentAlliance = Constants.Alliance.BLUE;
                        }
                    }

                    @Override
                    public void chooseRedAlliance() {
                        if (!configLocked) {
                            currentAlliance = Constants.Alliance.RED;
                        }
                    }

                    @Override
                    public void nextTaskChoice() {
                        driveTeamSignal.setForceEndGameState(true);

                        if (!configLocked) {
                            selectedTaskIndex++;

                            if (selectedTaskIndex > possibleTaskChoices.length - 1) { // why -1?
                                selectedTaskIndex = 0;
                            }
                        }
                    }

                    @Override
                    public void previousTaskChoice() {
                        driveTeamSignal.setForceEndGameState(true);

                        if (!configLocked) {
                            selectedTaskIndex--;

                            if (selectedTaskIndex < 0) {
                                selectedTaskIndex = possibleTaskChoices.length - 1; // why?
                            }
                        }
                    }

                    @Override
                    public void increaseDelay() {
                        driveTeamSignal.setForceEndGameState(true);

                        if (!configLocked) {
                            initialDelaySeconds += 1;

                            if (initialDelaySeconds > 25) {
                                initialDelaySeconds = 25;
                            }
                        }
                    }

                    @Override
                    public void decreaseDelay() {
                        driveTeamSignal.setForceEndGameState(true);

                        if (!configLocked) {
                            initialDelaySeconds -= 1;

                            if (initialDelaySeconds < 0) {
                                initialDelaySeconds = 0;
                            }
                        }
                    }
                }).build();
    }

    private void setupOperatorControls() {
        operatorControls = IntoTheDeepOperatorControls.builder().operatorGamepad(new NinjaGamePad(gamepad2))
                //.scoringMech(scoringMech)
                //.specimenMechanism(specimenMechanism)
                .noAutoSpecimenUpDown(true)
                .build();
    }
}