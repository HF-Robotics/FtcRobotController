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
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.season2324.Shared;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

import lombok.Getter;
import lombok.NonNull;

@Autonomous(name = "00 DECODE Auto", preselectTeleOp = DecodeDriverControlled.OP_MODE_NAME)
public class DecodeAuto extends OpMode {
    // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose RED_SCORE_POSE = new Pose(29.5, 144 - 33, Math.toRadians(135));

    public static final Pose RED_SCORE_SECOND_POSE = new Pose(58, 144 - 53, Math.toRadians(135));

    public static final Pose RED_SCORE_SECOND_LEAVE_POSE = new Pose(84, 144 - 53, Math.toRadians(135));

    public static final Pose BLUE_SCORE_POSE = new Pose(29.5, 33, Math.toRadians(225));

    public static final Pose BLUE_SCORE_SECOND_POSE = new Pose(58, 53, Math.toRadians(225));

    public static final Pose BLUE_SCORE_SECOND_LEAVE_POSE = new Pose(84, 53, Math.toRadians(225));

    public static final Pose RED_START_PACMAN_POSE = new Pose(82, 144 - 40, Math.toRadians(90));

    public static final Pose BLUE_START_PACMAN_POSE = new Pose(82, 40, Math.toRadians(270));

    public static final Pose RED_CLOSE_LEAVE_POSE = new Pose(38, 144 - 17, Math.toRadians(90));

    public static final Pose BLUE_CLOSE_LEAVE_POSE = new Pose(38, 17, Math.toRadians(270));

    private DecodeOperatorControls operatorControls;

    private Ticker ticker;

    private WheeledLauncher launcher;

    private GenevaCarousel carousel;

    private StateMachine stateMachine;

    private RollerIntake intake;

    private AprilTagAligner aprilTagAligner;

    private boolean canUseAprilTags;

    // FIXME: The tasks our robot knows how to do - rename these to
    //  something meaningful for the season!
    @Getter
    private enum Task {
        //PACMAN("(0) PACMAN"),
        GOAL_AND_45("(1) Goal and 45"),
        //GOAL_AND_45_MORE_SCORE("(2) 45 more score"),
        WALL_AND_GOAL("(2) Wall and goal"),
        //WALL_AND_GOAL_MORE_SCORE("(4) Wall and more score"),
        SIMPLE_LEAVE("(3) Simple leave"),
        SPACE_LAUNCHING("(5) Space Launching");

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

    private DecodeDriveTeamSignal driveTeamSignal;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            setupDriverControls();

            try {
                aprilTagAligner = new AprilTagAligner(telemetry, null, hardwareMap, ticker);
                canUseAprilTags = true;
            } catch (Exception ex) {
                Log.e(LOG_TAG, "Unable to initialize AprilTags", ex);
                canUseAprilTags = false;
            }

            ArtifactDetector artifactDetector;

            try {
                artifactDetector = new ArtifactDetector(hardwareMap, telemetry);
            } catch (IllegalArgumentException ex) {
                artifactDetector = null;
            }

            driveTeamSignal = new DecodeDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

            launcher = new WheeledLauncher(hardwareMap, telemetry, ticker, null);

            intake = new RollerIntake(hardwareMap);

            carousel = new AbsGenevaCarousel(hardwareMap, artifactDetector, intake, telemetry);

            pedroFollower = DecodeLargeDrivebasePedroConstants.createFollower(hardwareMap);

            setupOperatorControls();

            stateMachine = new StateMachine(telemetry);

            launcher.homeHood();

            // wait for hood to home, or timeout
            while (!launcher.isHoodIdle()) {
                launcher.periodicTask();
            }
        });
    }

    private void setupOperatorControls() {
        final NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

        operatorControls = DecodeOperatorControls.builder()
                .operatorGamepad(operatorGamepad)
                .intake(intake)
                .carousel(carousel).launcher(launcher).build();
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
            driveTeamSignal.setAlliance(currentAlliance);
            driveTeamSignal.periodicTask();
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
                setupGoalAndWallPath(false);
                break;
            case GOAL_AND_45:
                setupGoalFortyFivePath(false);
                break;
            case SIMPLE_LEAVE:
                setupSimpleLeavePath();
                break;
            case SPACE_LAUNCHING:
                setupSpaceLaunch();
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
                launcher.stopLauncher();
                launcher.lowerKicker(); // make sure this is lowered so it doesn't lower when we start tele-op!
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

    private void setupGoalAndWallPath(final boolean scoreMoreArtifacts) {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        sequenceOfStates.addRunnableStep("Pre-warm launcher", () -> launcher.closeLaunchVelocity());

        final double startPosY;
        final double startPoseX = 8.5;
        final Pose scorePose;
        final Pose startPacmanPose;
        final double startAndScoreHeadingDegrees;

        if (currentAlliance == Constants.Alliance.BLUE) {
            startPosY = 24 + 7.5; // FIXME - We don't understand this starting position!
            startAndScoreHeadingDegrees = 270;
            scorePose = new Pose(startPoseX+1, startPosY+24, Math.toRadians(startAndScoreHeadingDegrees + 2));
            startPacmanPose = BLUE_START_PACMAN_POSE;
        } else {
            startPosY = 144 - (24 + 7.5); // FIXME
            startAndScoreHeadingDegrees = 90;
            scorePose = new Pose(startPoseX+1, startPosY-24,  Math.toRadians(startAndScoreHeadingDegrees - 2));
            startPacmanPose = RED_START_PACMAN_POSE;
        }

        final Pose startPose = new Pose(startPoseX, startPosY, Math.toRadians(startAndScoreHeadingDegrees)); // Start Pose of our robot.

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        final Path scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pedroFollower.setStartingPose(startPose);

        PedroFollowerState scorePathState = new PedroFollowerState("Score preload", telemetry, pedroFollower, scorePreload);

        sequenceOfStates.addSequential(scorePathState);

        addLaunchSteps(sequenceOfStates, WheeledLauncher.TargetDistance.CLOSE);

        // This is where we need to choose to go pacman - or just leave

        if (scoreMoreArtifacts) {
            Pose midpointPose = new Pose(60 + 5, 144 - 60 + 5);

            final Path toEndPosePath = new Path(new BezierCurve(List.of(scorePose, midpointPose, startPacmanPose)));

            toEndPosePath.setLinearHeadingInterpolation(scorePose.getHeading(), startPacmanPose.getHeading());

            PedroFollowerState endPathState = new PedroFollowerState("End pose", telemetry, pedroFollower, toEndPosePath);

            sequenceOfStates.addSequential(endPathState);

            final Pose poseAtEndOfSecondScore = setupPacmanPath(sequenceOfStates, startPacmanPose);

            final Pose leaveEndPose;

            if (currentAlliance == Constants.Alliance.BLUE) {
                leaveEndPose = BLUE_SCORE_SECOND_LEAVE_POSE;
            } else {
                leaveEndPose = RED_SCORE_SECOND_LEAVE_POSE;
            }

            final Path toLeavePath = new Path(new BezierLine(poseAtEndOfSecondScore, leaveEndPose));

            toLeavePath.setLinearHeadingInterpolation(poseAtEndOfSecondScore.getHeading(), leaveEndPose.getHeading());

            PedroFollowerState leavePathState = new PedroFollowerState("Leave", telemetry, pedroFollower, toLeavePath);

            sequenceOfStates.addSequential(leavePathState);
        } else {
            addCloseLeavePath(scorePose, sequenceOfStates);
        }

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }

    private Pose setupPacmanPath(final SequenceOfStates sequenceOfStates, final Pose startPose) {
        final double direction;

        final Pose scorePose;

        if (currentAlliance == Constants.Alliance.BLUE) {
            scorePose = BLUE_SCORE_SECOND_POSE;
            direction = -1.0;
        } else {
            // Assume RED
            scorePose = RED_SCORE_SECOND_POSE;
            direction = 1.0;
        }

        sequenceOfStates.addRunnableStep("Intake on", () -> intake.intake());
        sequenceOfStates.addRunnableStep("Pre-warm launcher", () -> launcher.mediumLaunchVelocity());

        Pose nextArtifactPose = addOneArtifactIntakeSequence(sequenceOfStates, startPose, direction);
        nextArtifactPose = addOneArtifactIntakeSequence(sequenceOfStates, nextArtifactPose, direction);
        nextArtifactPose = addOneArtifactIntakeSequence(sequenceOfStates, nextArtifactPose, direction);

        // FIXME: Do we want to outtake here in case we ingested more than 3 somehow?
        //        *or* do we do this on every step in case something was not completely-ingested?

        sequenceOfStates.addRunnableStep("Intake off", () -> intake.stop());

        final Path scorePath = new Path(new BezierLine(nextArtifactPose, scorePose));

        scorePath.setLinearHeadingInterpolation(nextArtifactPose.getHeading(), scorePose.getHeading());

        PedroFollowerState scorePathState = new PedroFollowerState("Score again!", telemetry, pedroFollower, scorePath);
        sequenceOfStates.addSequential(scorePathState);

        addLaunchSteps(sequenceOfStates, WheeledLauncher.TargetDistance.MEDIUM);

        return scorePose;
    }

    private Pose addOneArtifactIntakeSequence(final SequenceOfStates sequenceOfStates,
                                              final Pose startFromPose, final double direction) {
        final Pose toNextArtifactPose = new Pose(startFromPose.getX(), startFromPose.getY() + (direction * 5), startFromPose.getHeading());
        final Path intakeNextArtifactPath = new Path(new BezierLine(startFromPose, toNextArtifactPose));
        intakeNextArtifactPath.setLinearHeadingInterpolation(startFromPose.getHeading(), toNextArtifactPose.getHeading());
        PedroFollowerState intakeNextArtifactPathState = new PedroFollowerState("Intake next artifact", telemetry, pedroFollower, intakeNextArtifactPath);

        final State nextIntakeIndexState = carousel.nextIntakeIndexState(telemetry, ticker, false);
        sequenceOfStates.addSequential(nextIntakeIndexState);
        sequenceOfStates.addSequential(intakeNextArtifactPathState);

        // FIXME: Let's see how low we can push this?
        sequenceOfStates.addWaitStep("No jam", 500, TimeUnit.MILLISECONDS);

        return toNextArtifactPose;
    }

    private void setupGoalFortyFivePath(final boolean scoreMoreArtifacts) {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        sequenceOfStates.addRunnableStep("Pre-warm launcher", () -> launcher.closeLaunchVelocity());

        final double startAngle;
        final double startPosY;
        final double startPosX;

        final Pose scorePose;
        final Pose startPacmanPose;

        if (currentAlliance == Constants.Alliance.BLUE) {
            startPosY = 22;
            startPosX = 16.5;
            startAngle = Math.toRadians(225);

            scorePose = BLUE_SCORE_POSE;
            startPacmanPose = BLUE_START_PACMAN_POSE;
        } else {
            startAngle = Math.toRadians(225 - 90);
            startPosY = 144 - 22; // FIXME
            startPosX = 16.5;

            scorePose = RED_SCORE_POSE;
            startPacmanPose = RED_START_PACMAN_POSE;
        }

        final Pose startPose = new Pose(startPosX, startPosY, startAngle); // Start Pose of our robot.

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        final Path scorePreload = new Path(new BezierLine(startPose, scorePose));

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pedroFollower.setStartingPose(startPose);

        PedroFollowerState scorePathState = new PedroFollowerState("Score preload", telemetry, pedroFollower, scorePreload);

        sequenceOfStates.addSequential(scorePathState);

        addLaunchSteps(sequenceOfStates, WheeledLauncher.TargetDistance.CLOSE);

        // Here is where we decide to score more or leave
        if (scoreMoreArtifacts) {
            final Path toEndPosePath = new Path(new BezierLine(scorePose, startPacmanPose));

            toEndPosePath.setLinearHeadingInterpolation(scorePose.getHeading(), startPacmanPose.getHeading());

            PedroFollowerState endPathState = new PedroFollowerState("End pose", telemetry, pedroFollower, toEndPosePath);

            sequenceOfStates.addSequential(endPathState);

            final Pose poseAtEndOfSecondScore = setupPacmanPath(sequenceOfStates, startPacmanPose);

            final Pose leaveEndPose;

            if (currentAlliance == Constants.Alliance.BLUE) {
                leaveEndPose = BLUE_SCORE_SECOND_LEAVE_POSE;
            } else {
                leaveEndPose = RED_SCORE_SECOND_LEAVE_POSE;
            }

            final Path toLeavePath = new Path(new BezierLine(poseAtEndOfSecondScore, leaveEndPose));

            toLeavePath.setLinearHeadingInterpolation(poseAtEndOfSecondScore.getHeading(), leaveEndPose.getHeading());

            PedroFollowerState leavePathState = new PedroFollowerState("Leave", telemetry, pedroFollower, toLeavePath);

            sequenceOfStates.addSequential(leavePathState);
        } else {
            addCloseLeavePath(scorePose, sequenceOfStates);
        }

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }

    private void addCloseLeavePath(Pose scorePose, SequenceOfStates sequenceOfStates) {
        final Pose leaveEndPose;

        if (currentAlliance == Constants.Alliance.BLUE) {
            leaveEndPose = BLUE_CLOSE_LEAVE_POSE;
        } else {
            leaveEndPose = RED_CLOSE_LEAVE_POSE;
        }

        final Path toLeavePath = new Path(new BezierLine(scorePose, leaveEndPose));

        toLeavePath.setLinearHeadingInterpolation(scorePose.getHeading(), leaveEndPose.getHeading());

        PedroFollowerState leavePathState = new PedroFollowerState("Leave", telemetry, pedroFollower, toLeavePath);

        sequenceOfStates.addSequential(leavePathState);
    }

    private void setupSpaceLaunch() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        sequenceOfStates.addRunnableStep("Pre-warm launcher", () -> launcher.farLaunchVelocity());

        final double startPosY;
        final double startPoseX = 135;
        final double scorePoseY;
        final double scorePoseX = 132;

        final double scoreHeadingDegrees;

        double angleTowardsTargetDegrees = 24;

        if (currentAlliance == Constants.Alliance.RED) {
            angleTowardsTargetDegrees = -angleTowardsTargetDegrees;
        }

        // Since we have not moved yet, we can compute the initial heading
        // to use if we have a bearing from the april tags
        if (canUseAprilTags) {
            final Double detectedBearingInDegrees = aprilTagAligner.getBearing(currentAlliance);

            if (detectedBearingInDegrees != null) {
                driveTeamSignal.signalGoalAcquired();

                angleTowardsTargetDegrees = detectedBearingInDegrees;
                Log.d(LOG_TAG, "Setting angle to target based on april tags to " + angleTowardsTargetDegrees);
            } else {
                Log.e(LOG_TAG, "April tags working, but no usable tag found to use for aiming");
            }
        }

        double initialRobotHeadingDegrees = 180;

        scoreHeadingDegrees = initialRobotHeadingDegrees + angleTowardsTargetDegrees;

        if (currentAlliance == Constants.Alliance.BLUE) {
            startPosY = 60;
        } else {
            startPosY = 84;
        }
        scorePoseY = startPosY;

        final Pose startPose = new Pose(startPoseX, startPosY, Math.toRadians(initialRobotHeadingDegrees)); // Start Pose of our robot.

        final Pose scorePose = new Pose(scorePoseX, scorePoseY, Math.toRadians(scoreHeadingDegrees));

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        final Path scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pedroFollower.setStartingPose(startPose);

        PedroFollowerState scorePathState = new PedroFollowerState("Score preload", telemetry,
                pedroFollower, scorePreload);

        sequenceOfStates.addSequential(scorePathState);

        // Need more time for launcher to come to speed
        sequenceOfStates.addWaitStep("Launcher settling", 2, TimeUnit.SECONDS);

        addLaunchSteps(sequenceOfStates, WheeledLauncher.TargetDistance.FAR);

        // This is where we need to choose to go pacman - or just leave

        final Pose leaveEndPose;

        if (currentAlliance == Constants.Alliance.BLUE) {
            leaveEndPose = new Pose(144 - 9.5, 36, Math.toRadians(initialRobotHeadingDegrees));
        } else {
            leaveEndPose = new Pose(144 - 9.5, 144 - 36, Math.toRadians(initialRobotHeadingDegrees));
        }

        final Path toLeavePath = new Path(new BezierLine(scorePose, leaveEndPose));

        toLeavePath.setLinearHeadingInterpolation(scorePose.getHeading(), leaveEndPose.getHeading());

        PedroFollowerState leavePathState = new PedroFollowerState("Leave", telemetry, pedroFollower, toLeavePath);

        sequenceOfStates.addSequential(leavePathState);

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }

    private void setupSimpleLeavePath() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        final double yPosition;

        if (currentAlliance == Constants.Alliance.BLUE) {
            yPosition = -24;
        } else {
            yPosition = 24;
        }

        final Pose endPose = new Pose(144 - 1, yPosition, Math.toRadians(180));

        final Pose startPose = new Pose(144, 0, Math.toRadians(180)); // Start Pose of our robot.

        final Path driveForward = new Path(new BezierLine(startPose, endPose));
        driveForward.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        pedroFollower.setStartingPose(startPose);

        PedroFollowerState driveForwardPathState = new PedroFollowerState("Drive forward", telemetry, pedroFollower, driveForward);
        sequenceOfStates.addSequential(driveForwardPathState);

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }
    private void addLaunchSteps(final SequenceOfStates sequenceOfStates, final WheeledLauncher.TargetDistance targetDistance) {
        addOneLaunch(sequenceOfStates, targetDistance, true); // save time for the first launch
        addOneLaunch(sequenceOfStates, targetDistance, false);
        addOneLaunch(sequenceOfStates, targetDistance, false);
    }

    private void addOneLaunch(SequenceOfStates sequenceOfStates, final WheeledLauncher.TargetDistance targetDistance, final boolean skipMoveIfInPosition) {
        LauncherToSpeedState toSpeedState = new LauncherToSpeedState(telemetry, ticker, targetDistance);

        State carouselNextLaunchIndexState = carousel.nextLaunchIndexState(telemetry, ticker, skipMoveIfInPosition);

        sequenceOfStates.addSequential(carouselNextLaunchIndexState);
        sequenceOfStates.addSequential(toSpeedState);
        sequenceOfStates.addRunnableStep("Raise kicker", () -> launcher.raiseKickerNoMatterWhat());
        sequenceOfStates.addWaitStep("Wait raise kicker", 750, TimeUnit.MILLISECONDS);
        sequenceOfStates.addRunnableStep("Lower kicker", () -> launcher.lowerKicker());
        sequenceOfStates.addWaitStep("Wait lower kicker", 300, TimeUnit.MILLISECONDS);
    }

    class LauncherToSpeedState extends StopwatchTimeoutSafetyState {
        private final WheeledLauncher.TargetDistance distance;

        protected LauncherToSpeedState(final Telemetry telemetry, final Ticker ticker, WheeledLauncher.TargetDistance distance) {
            super("Speeding up", telemetry, ticker, 5_000);
            this.distance = distance;
        }

        @Override
        public State doStuffAndGetNextState() {
            switch (distance) {
                case CLOSE:
                    launcher.closeLaunchVelocity();
                    break;
                case MEDIUM:
                    launcher.mediumLaunchVelocity();
                    break;
                case FAR:
                    launcher.farLaunchVelocity();
                    break;
            }

            launcher.periodicTask();

            String launcherState = "!";
            String hoodState = "_";

            if (launcher.isAtTargetVelocity()) {
                Log.d(LOG_TAG, "Launcher speed is ready");
                launcherState = "G";

                if (launcher.isHoodIdle()) {
                    hoodState = "/";

                    resetToStart();

                    return nextState;
                } else {
                    Log.d(LOG_TAG, "Hood is not in position");
                }
            }

            telemetry.addData("Launcher", launcherState + ":" + hoodState);

            if (isTimedOut()) {
                resetToStart();

                Log.e(LOG_TAG, "Timed out waiting for speed or hood");

                return nextState;
            }

            return this;
        }

        @Override
        public void resetToStart() {

        }
    }
}