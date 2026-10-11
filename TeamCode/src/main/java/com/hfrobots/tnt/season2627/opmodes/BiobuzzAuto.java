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
import static com.hfrobots.tnt.corelib.Constants.Alliance.BLUE;
import static com.hfrobots.tnt.corelib.Constants.Alliance.RED;
import static com.pedropathing.api.Paths.line;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.pedro.HoldPositionState;
import com.hfrobots.tnt.corelib.drive.pedro.PedroFollowerState;
import com.hfrobots.tnt.season2324.Shared;
import com.hfrobots.tnt.season2526.drivebase.DecodeLargeDrivebasePedroConstants;
import com.hfrobots.tnt.season2526.driveteam.DecodeOperatorControls;
import com.hfrobots.tnt.season2526.mechanisms.WheeledLauncher;
import com.hfrobots.tnt.season2627.driveteam.BiobuzzDriverControls;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Getter;
import lombok.NonNull;

@Autonomous(name = "00 Biobuzz Auto", preselectTeleOp = BiobuzzDriverControlled.OP_MODE_NAME)
public class BiobuzzAuto extends OpMode {
    private DecodeOperatorControls operatorControls;

    private Ticker ticker;

    private StateMachine stateMachine;

    // FIXME: The tasks our robot knows how to do - rename these to
    //  something meaningful for the season!
    @Getter
    private enum Task {
        LEAVE("Leave and Park");

        final String description;

        Task(String description) {
            this.description = description;
        }

    }

    private int selectedTaskIndex = 0;

    private final Task[] possibleTaskChoices = Task.values();

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Constants.Alliance currentAlliance = Constants.Alliance.RED;

    private int initialDelaySeconds = 0;

    private Follower pedroFollower;

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    private BiobuzzDriverControls driverControls;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            setupDriverControls();

            pedroFollower = DecodeLargeDrivebasePedroConstants.create(hardwareMap);

            setupOperatorControls();

            stateMachine = new StateMachine(telemetry);
        });
    }

    private void setupOperatorControls() {
        final NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

        operatorControls = DecodeOperatorControls.builder()
                .operatorGamepad(operatorGamepad).build();
    }

    @Override
    public void start() {
        Shared.withBetterErrorHandling(() -> {
            super.start();

            setupStateMachine();
        });
    }

    public void stop() {
        Shared.withBetterErrorHandling(super::stop);
    }

    private boolean configLocked = false;

    @Override
    public void init_loop() {
        doAutoConfig();
        operatorControls.periodicTask();

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
            //driveTeamSignal.setAlliance(currentAlliance);
            //driveTeamSignal.periodicTask();
        });
    }

    @Override
    public void loop() {
        try {
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
                throw (RuntimeException) t;
            }

            RuntimeException rte = new RuntimeException();
            rte.initCause(t);

            throw rte;
        }
    }

    private void setupStateMachine() {
        Task selectedTask = possibleTaskChoices[selectedTaskIndex];

        // FIXME: Change the methods in the switch() below to align with
        // the name of each task
        switch (selectedTask) {
            case LEAVE:
                setupLeavePark();
                break;
            default:
                stateMachine.addSequential(new DoneState("Default done", telemetry));
                break;
        }

        if (initialDelaySeconds != 0) {
            stateMachine.addStartDelay(initialDelaySeconds, Ticker.systemTicker());
        }
    }

    class DoneState extends State {

        private boolean issuedStop = false;

        public DoneState(@NonNull String name, Telemetry telemetry) {
            super(name, telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (!issuedStop) {
                issuedStop = true;
            }

            return nextState;
        }

        @Override
        public void resetToStart() {
            issuedStop = false;
        }
    }

    /**
     * Creates an instance of the "done" state which stops the robot and should be the
     * "end" state of all of our robot's state machines
     */
    protected void commonCompletionStates(final SequenceOfStates sequenceOfStates) {
        sequenceOfStates.addSequential(new DoneState("Stop mechanisms", telemetry));
        sequenceOfStates.addSequential(new HoldPositionState("Parked", telemetry, pedroFollower));
    }

    private void setupDriverControls() {
        driverControls = BiobuzzDriverControls.builder()
                .driversGamepad(new NinjaGamePad(gamepad1)).autoConfigTask(new BiobuzzDriverControls.InitLoopConfigTask() {
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
                            currentAlliance = BLUE;
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

    private void setupLeavePark() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        final double startPoseX;

        final double startPoseY;

        final double startHeadingDegrees;

        final double endPoseX;

        final double endPoseY;

        final double endHeadingDegrees;

        if (currentAlliance == RED) {
            startPoseX = 60;

            startPoseY = 144;

            startHeadingDegrees = 180;

            endPoseX = 0;

            endPoseY = 132;

            endHeadingDegrees = 180;
        } else {
            startPoseX = 84;

            startPoseY = 0;

            startHeadingDegrees = 0;

            endPoseX = 144;

            endPoseY = 6;

            endHeadingDegrees = 0;
        }


        final Pose startPose = poseFactory.of(startPoseX, startPoseY, startHeadingDegrees); // Start Pose of our robot.

        final Pose endPose = poseFactory.of(endPoseX, endPoseY, endHeadingDegrees);

        final Path parkPath = line(startPose, endPose).linear(startPose, endPose);

        pedroFollower.setPose(startPose);

        PedroFollowerState parkPathState = new PedroFollowerState("leave and park", telemetry, pedroFollower, parkPath);

        sequenceOfStates.addSequential(parkPathState);

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }

    private void addLaunchSteps(final SequenceOfStates sequenceOfStates, final WheeledLauncher.TargetDistance targetDistance) {
        addOneLaunch(sequenceOfStates, targetDistance, true); // save time for the first launch
        addOneLaunch(sequenceOfStates, targetDistance, false);
        addOneLaunch(sequenceOfStates, targetDistance, false);
    }

    private void addOneLaunch(SequenceOfStates sequenceOfStates, final WheeledLauncher.TargetDistance targetDistance, final boolean skipMoveIfInPosition) {


    }
}