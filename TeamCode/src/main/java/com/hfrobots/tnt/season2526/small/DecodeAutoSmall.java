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

package com.hfrobots.tnt.season2526.small;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;
import android.util.Size;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.season2324.Shared;
import com.hfrobots.tnt.season2526.opmodes.DecodeDriverControlled;
import com.hfrobots.tnt.season2526.driveteam.DecodeDriverControls;
import com.hfrobots.tnt.season2526.drivebase.PedroFollowerState;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

import lombok.Getter;

@Autonomous(name = "00 DECODE Auto Small", preselectTeleOp = DecodeDriverControlled.OP_MODE_NAME)
@Disabled
public class DecodeAutoSmall extends OpMode {
    // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose RED_SCORE_POSE = new Pose(53.5, 144 - 59, Math.toRadians(225 - 90));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose BLUE_SCORE_POSE = new Pose(53.5, 59, Math.toRadians(225));

    public static final double SCORE_POSE_LAUNCHER_VELOCITY = 2060;

    private Ticker ticker;


    private StateMachine stateMachine;

    // FIXME: The tasks our robot knows how to do - rename these to
    //  something meaningful for the season!
    @Getter
    private enum Task {
        WALL_AND_GOAL("Wall and goal"),
        GOAL_AND_45("Goal and 45");

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

    private DecodeDriverControls driverControls;

    private VisionPortal visionPortal;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            setupDriverControls();

            //setupVisionPortal(hardwareMap);

            // driveTeamSignal = new IntoTheDeepDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

            pedroFollower = DecodeSmallDrivebasePedroConstants.createFollower(hardwareMap);

            stateMachine = new StateMachine(telemetry);
        });
    }

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
            if (visionPortal != null) {
                visionPortal.stopLiveView();
            }

            setupStateMachine();
        });
    }

    public void stop() {
        Shared.withBetterErrorHandling(() -> {
            super.stop();
            if (visionPortal != null) {
                visionPortal.stopStreaming();
                visionPortal.stopLiveView();
            }
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

            telemetry.addData("01", "Alliance: %s", currentAlliance);
            telemetry.addData("02", "Task: %s", possibleTaskChoices[selectedTaskIndex].getDescription());
            telemetry.addData("03", "Delay %d sec", initialDelaySeconds);
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
                throw (RuntimeException)t;
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
            case WALL_AND_GOAL:
                setupGoalAndWallPath();
                break;
            case GOAL_AND_45:
                setupGoalFortyFivePath();
                break;
            default:
                stateMachine.addSequential(newDoneState("Default done"));
                break;
        }

        if (initialDelaySeconds != 0) {
            stateMachine.addStartDelay(initialDelaySeconds, Ticker.systemTicker());
        }
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
                    pedroFollower.pausePathFollowing();
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
        driverControls = DecodeDriverControls.builder()
                .driversGamepad(new NinjaGamePad(gamepad1)).autoConfigTask(new DecodeDriverControls.InitLoopConfigTask() {
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

    private void setupGoalAndWallPath() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        final double startPosY;
        final Pose scorePose;

        if (currentAlliance == Constants.Alliance.BLUE) {
            startPosY = 33 - 14; // FIXME - We don't understand this starting position!
            scorePose = BLUE_SCORE_POSE;
        } else {
            startPosY = 144 - (33 - 14); // FIXME
            scorePose = RED_SCORE_POSE;
        }

        final Pose startPose = new Pose(0, startPosY, Math.toRadians(180)); // Start Pose of our robot.

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        final Path scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pedroFollower.setStartingPose(startPose);

        PedroFollowerState scorePathState = new PedroFollowerState("Score preload", telemetry, pedroFollower, scorePreload);

        sequenceOfStates.addSequential(scorePathState);
        sequenceOfStates.addSequential(newDoneState("Done!"));
        stateMachine.addSequence(sequenceOfStates);
    }

    private void setupGoalFortyFivePath() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        final double startAngle;
        final double startPosY;
        final double startPosX;

        final Pose scorePose;

        if (currentAlliance == Constants.Alliance.BLUE) {
            startPosY = 22;
            startPosX = 16.5;
            startAngle = Math.toRadians(225);

            scorePose = BLUE_SCORE_POSE;
        } else {
            startAngle = Math.toRadians(225 - 90);
            startPosY = 144 - 22; // FIXME
            startPosX = 16.5;

            scorePose = RED_SCORE_POSE;
        }

        final Pose startPose = new Pose(startPosX, startPosY, startAngle); // Start Pose of our robot.

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        final Path scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pedroFollower.setStartingPose(startPose);

        PedroFollowerState scorePathState = new PedroFollowerState("Score preload", telemetry, pedroFollower, scorePreload);

        sequenceOfStates.addSequential(scorePathState);
        sequenceOfStates.addSequential(newDoneState("Done!"));
        stateMachine.addSequence(sequenceOfStates);
    }
}