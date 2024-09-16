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

package com.hfrobots.tnt.util.templates;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;
import android.util.Size;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.google.common.base.Optional;
import com.google.common.base.Ticker;
import com.google.common.collect.Sets;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.state.ReadyCheckable;
import com.hfrobots.tnt.season2324.Shared;
import com.hfrobots.tnt.util.DriveTeamSignal;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Set;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name="00 [SEASON NAME] Auto")
public class TemplateAuto extends OpMode {
    private Ticker ticker;

    // private RoadRunnerMecanumDriveBase driveBase;

    private StateMachine stateMachine;

    // Used to ensure that states with circular dependencies are setup correctly
    private final Set<ReadyCheckable> readyCheckables = Sets.newHashSet();

    // FIXME: The tasks our robot knows how to do - rename these to
    //  something meaningful for the season!
    private enum Task {
        TASK_CHOICE_A("Choice A"),
        TASK_CHOICE_B("Choice B"),
        TASK_CHOICE_C("Choice C");

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

    // private ExampleDriveConstants driveConstants;

    private TemplateOperatorControls operatorControls;

    private TemplateDriverControls driverControls;

    private VisionPortal visionPortal;

    private DriveTeamSignal driveTeamSignal;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            // FIXME: Setup mechanisms from hardware map here

            setupDriverControls();
            setupOperatorControls();
            setupVisionPortal(hardwareMap);

            driveTeamSignal = new DriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

            // driveConstants = new ExampleDriveConstants();

            IMU.Parameters imuParameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.DOWN));

            // driveBase = new RoadRunnerMecanumDriveBase(hardwareMap,
            //        driveConstants, Optional.of(imuParameters));

            stateMachine = new StateMachine(telemetry);
        });
    }

    private com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera pipelineAndCamera;

    private void setupVisionPortal(final HardwareMap hardwareMap) {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        // FIXME: Need to add processor(s) here for the vision portal to actually
        // detect anything!

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    @Override
    public void start() {
        Shared.withBetterErrorHandling(() -> {
            super.start();
            visionPortal.stopLiveView();
        });
    }

    public void stop() {
        Shared.withBetterErrorHandling(() -> {
            super.stop();
            visionPortal.stopStreaming();
            visionPortal.stopLiveView();
        });
    }

    private boolean configLocked = false;

    @Override
    public void init_loop() {
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

            if (operatorControls != null) {
                operatorControls.periodicTask();
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
        try {
            if (!stateMachineSetup) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                Task selectedTask = possibleTaskChoices[selectedTaskIndex];

                // FIXME: Change the methods in the switch() below to align with
                // the name of each task
                switch (selectedTask) {
                    case TASK_CHOICE_A:
                        setupTaskChoiceA();
                        break;
                    case TASK_CHOICE_B:
                        setupTaskChoiceB();
                        break;
                    case TASK_CHOICE_C:
                        setupTaskChoiceC();
                        break;
                    default:
                        stateMachine.addSequential(newDoneState("Default done"));
                        break;
                }

                if (initialDelaySeconds != 0) {
                    stateMachine.addStartDelay(initialDelaySeconds, Ticker.systemTicker());
                }

                stateMachineSetup = true;
            }

            stateMachine.doOneStateLoop();

            // If you have other mechanisms, like a DriveTeamSignal that needs
            // to run, do it here
            //
            // driveTeamSignal.periodicTask();

            telemetry.update(); // send all telemetry to the drivers' station
        } catch (Throwable t) {
            // Better logging than the FTC SDK provides :(
            Log.e(LOG_TAG, "Exception during state machine", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException)t;
            }

            RuntimeException rte = new RuntimeException();
            rte.initCause(t);

            throw rte;
        }
    }

    protected void setupTaskChoiceA() {
        // Different ways to do this, if complex transitions involved, then
        // chain to first state, and then set that as the first state on the state
        // machine:

        final State firstState = null; // FIXME: Replace with something real
        stateMachine.setFirstState(firstState);

        // Alternatively, for something straightforward you can do sequentials, like this:
        SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);
        sequenceOfStates.addSequential(null /* Use your State */);
        sequenceOfStates.addWaitStep("Wait 2 seconds", 2, TimeUnit.SECONDS);
        stateMachine.addSequence(sequenceOfStates);
    }

    void setupTaskChoiceB() {

    }

    void setupTaskChoiceC() {

    }

    protected State newMsDelayState(String name, final int numberOfMillis) {
        return new StopwatchDelayState(name, telemetry, ticker, numberOfMillis, TimeUnit.MILLISECONDS);
    }

    /**
     * Creates an instance of the "done" state which stops the robot and should be the
     * "end" state of all of our robot's state machines
     */
    protected State newDoneState(String name) {
        return new State(name, telemetry) {
            private boolean issuedStop = false;

            @Override
            public State doStuffAndGetNextState() {
                // FIXME: Stop everything on the robot here
                if (!issuedStop) {
                    // driveBase.setMotorPowers(0, 0, 0, 0);

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

    private void setupDriverControls() {
        driverControls = TemplateDriverControls.builder()
                .driversGamepad(new NinjaGamePad(gamepad1)).autoConfigTask(new TemplateDriverControls.InitLoopConfigTask() {
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
                        if (!configLocked) {
                            selectedTaskIndex++;

                            if (selectedTaskIndex > possibleTaskChoices.length - 1) { // why -1?
                                selectedTaskIndex = 0;
                            }
                        }
                    }

                    @Override
                    public void previousTaskChoice() {
                        if (!configLocked) {
                            selectedTaskIndex--;

                            if (selectedTaskIndex < 0) {
                                selectedTaskIndex = possibleTaskChoices.length - 1; // why?
                            }
                        }
                    }

                    @Override
                    public void increaseDelay() {
                        if (!configLocked) {
                            initialDelaySeconds += 1;

                            if (initialDelaySeconds > 25) {
                                initialDelaySeconds = 25;
                            }
                        }
                    }

                    @Override
                    public void decreaseDelay() {
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
        operatorControls = TemplateOperatorControls.builder().operatorGamepad(new NinjaGamePad(gamepad2)).build();
    }
}