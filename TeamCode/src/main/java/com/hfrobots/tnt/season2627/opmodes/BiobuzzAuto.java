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

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.pedro.HoldPositionState;
import com.hfrobots.tnt.corelib.drive.pedro.PedroFollowerState;
import com.hfrobots.tnt.season2324.Shared;
import com.hfrobots.tnt.season2526.driveteam.DecodeOperatorControls;
import com.hfrobots.tnt.season2627.BiobuzzPedroConstants;
import com.hfrobots.tnt.season2627.FieldCoordinates;
import com.hfrobots.tnt.season2627.driveteam.BiobuzzDriveTeamSignal;
import com.hfrobots.tnt.season2627.driveteam.BiobuzzDriverControls;

import com.hfrobots.tnt.season2627.mechanisms.RollerIntake;
import com.hfrobots.tnt.season2627.mechanisms.WheeledLauncher;
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

@Autonomous(name = "00 Biobuzz Auto", preselectTeleOp = BiobuzzDriverControlled.OP_MODE_NAME)
public class BiobuzzAuto extends OpMode {

    private static final double ROBOT_WIDTH_INCHES = 18;

    private static final double ROBOT_LENGTH_INCHES = 18;

    private DecodeOperatorControls operatorControls;

    private Ticker ticker;

    private StateMachine stateMachine;

    private WheeledLauncher wheeledLauncher;

    private RollerIntake intake;

    // FIXME: The tasks our robot knows how to do - rename these to
    //  something meaningful for the season!
    @Getter
    private enum Task {
        LEAVE("Leave and Park"),
        LAUNCH_AND_LEAVE("Launch and park"),

        FLOWER_LAUNCH("Flower side launch");

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

    private BiobuzzDriverControls driverControls;

    private BiobuzzDriveTeamSignal signal;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            setupDriverControls();

            pedroFollower = BiobuzzPedroConstants.createFollower(hardwareMap);

            setupOperatorControls();

            stateMachine = new StateMachine(telemetry);

            wheeledLauncher = new WheeledLauncher(hardwareMap, telemetry, ticker);
            intake = new RollerIntake(hardwareMap);

            signal = new BiobuzzDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);
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
            signal.setAlliance(currentAlliance);
            signal.periodicTask();
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
            case LAUNCH_AND_LEAVE:
                setupLaunchLeavePark();
                break;
            case FLOWER_LAUNCH:
                setupFlowerSideLaunch();
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
                if (wheeledLauncher != null) {
                    wheeledLauncher.stopLauncher();
                }

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

    final static double PARK_DISTANCE_OFF_WALL = 14;

    private void setupLeavePark() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        final double startPoseX;

        final double startPoseY;

        final double frontOfFlowerY;

        final double startHeadingDegrees;

        final double endPoseX;

        final double endPoseY;

        final double endHeadingDegrees;

        if (currentAlliance == RED) {
            startPoseX = FieldCoordinates.RED_HIVE_CENTER_LINE.x; // align center of bot with hive

            startPoseY = FieldCoordinates.FLOWER_TOP_OF_FIELD.y - (ROBOT_LENGTH_INCHES / 2);

            frontOfFlowerY = startPoseY - 10; // would need to be 12, and should be a constant for launching

            startHeadingDegrees = FieldCoordinates.AWAY_FROM_AUDIENCE;

            endPoseX = FieldCoordinates.LOADING_ZONE_RED_YCENTER_AGAINST_THE_WALL.x + PARK_DISTANCE_OFF_WALL;

            endPoseY = FieldCoordinates.LOADING_ZONE_RED_YCENTER_AGAINST_THE_WALL.y + 13 + 9;

            endHeadingDegrees = FieldCoordinates.AWAY_FROM_AUDIENCE;
        } else {
            startPoseX = FieldCoordinates.BLUE_HIVE_CENTER_LINE.x;

            startPoseY = FieldCoordinates.FLOWER_BOTTOM_OF_FIELD.y + (ROBOT_LENGTH_INCHES / 2);

            frontOfFlowerY = startPoseY + 10; // would need to be 12, and should be a constant for launching

            startHeadingDegrees = FieldCoordinates.TOWARDS_AUDIENCE;

            endPoseX = FieldCoordinates.LOADING_ZONE_BLUE_YCENTER_AGAINST_THE_WALL.x - PARK_DISTANCE_OFF_WALL;

            endPoseY = FieldCoordinates.LOADING_ZONE_BLUE_YCENTER_AGAINST_THE_WALL.y -13 - 9;

            endHeadingDegrees = FieldCoordinates.TOWARDS_AUDIENCE;
        }


        final Pose startPose = new Pose(startPoseX, startPoseY,  Math.toRadians(startHeadingDegrees)); // Start Pose of our robot.

        final Pose clearOfFlower = poseOfDegrees(startPoseX, frontOfFlowerY, startHeadingDegrees);
        final Pose endPose = new Pose(endPoseX, endPoseY, Math.toRadians(endHeadingDegrees));

        final Path clearOfFlowerPath  = line(startPose, clearOfFlower);
        linear(clearOfFlowerPath, startPose, clearOfFlower);

        final Path parkPath = line(clearOfFlower, endPose);
        linear(parkPath, clearOfFlower, endPose);

        pedroFollower.setPose(startPose);

        PedroFollowerState clearOfFlowerState = new PedroFollowerState("Getting clear of flower", telemetry, pedroFollower, clearOfFlowerPath);
        sequenceOfStates.addSequential(clearOfFlowerState);

        PedroFollowerState parkPathState = new PedroFollowerState("leave and park", telemetry, pedroFollower, parkPath);

        sequenceOfStates.addSequential(parkPathState);

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }

    private void setupFlowerSideLaunch() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        final double startPoseX;

        final double startPoseY;

        final double frontOfFlowerY;

        final double startHeadingDegrees;

        final double endPoseX;

        final double endPoseY;

        final double endHeadingDegrees;

        if (currentAlliance == RED) {
            startPoseX = FieldCoordinates.RED_HIVE_CENTER_LINE.x; // align center of bot with hive

            startPoseY = FieldCoordinates.FLOWER_TOP_OF_FIELD.y - (ROBOT_LENGTH_INCHES / 2);

            frontOfFlowerY = startPoseY - 12; // would need to be 12, and should be a constant for launching

            startHeadingDegrees = FieldCoordinates.AWAY_FROM_AUDIENCE;

            endPoseX = FieldCoordinates.LOADING_ZONE_RED_YCENTER_AGAINST_THE_WALL.x + PARK_DISTANCE_OFF_WALL;

            endPoseY = FieldCoordinates.LOADING_ZONE_RED_YCENTER_AGAINST_THE_WALL.y + 13 + 6;

            endHeadingDegrees = FieldCoordinates.AWAY_FROM_AUDIENCE;
        } else {
            startPoseX = FieldCoordinates.BLUE_HIVE_CENTER_LINE.x;

            startPoseY = FieldCoordinates.FLOWER_BOTTOM_OF_FIELD.y + (ROBOT_LENGTH_INCHES / 2);

            frontOfFlowerY = startPoseY + 12; // would need to be 12, and should be a constant for launching

            startHeadingDegrees = FieldCoordinates.TOWARDS_AUDIENCE;

            endPoseX = FieldCoordinates.LOADING_ZONE_BLUE_YCENTER_AGAINST_THE_WALL.x - PARK_DISTANCE_OFF_WALL;

            endPoseY = FieldCoordinates.LOADING_ZONE_BLUE_YCENTER_AGAINST_THE_WALL.y -13 - 6;

            endHeadingDegrees = FieldCoordinates.TOWARDS_AUDIENCE;
        }


        final Pose startPose = new Pose(startPoseX, startPoseY,  Math.toRadians(startHeadingDegrees)); // Start Pose of our robot.

        final Pose clearOfFlower = poseOfDegrees(startPoseX, frontOfFlowerY, startHeadingDegrees);
        final Pose endPose = new Pose(endPoseX, endPoseY, Math.toRadians(endHeadingDegrees));

        final Path clearOfFlowerPath  = line(startPose, clearOfFlower);
        linear(clearOfFlowerPath, startPose, clearOfFlower);

        final Path parkPath = line(clearOfFlower, endPose);
        linear(parkPath, clearOfFlower, endPose);

        // Always delay (until we can detect hive tips)
        sequenceOfStates.addWaitStep("waitForPartner", 15, TimeUnit.SECONDS);
        preWarmLauncer(sequenceOfStates);
        pedroFollower.setPose(startPose);

        PedroFollowerState clearOfFlowerState = new PedroFollowerState("Getting clear of flower", telemetry, pedroFollower, clearOfFlowerPath);
        sequenceOfStates.addSequential(clearOfFlowerState);

        addLaunchSteps(sequenceOfStates);

        PedroFollowerState parkPathState = new PedroFollowerState("leave and park", telemetry, pedroFollower, parkPath);

        sequenceOfStates.addSequential(parkPathState);

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }

    private void linear(final Path path, final Pose startPose, final Pose endPose) {
        path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    }

    private void setupLaunchLeavePark() {
        final SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        final double startPoseX;

        final double startPoseY;

        final double startHeadingDegrees;

        final double endPoseX;

        final double endPoseY;

        final double endHeadingDegrees;

        final double launchPoseY;

        final double midpointCurveX;

        final double midpointCurveY;

        if (currentAlliance == RED) {
            startPoseX = FieldCoordinates.RED_HIVE_CENTER_LINE.x; // align center of bot with hive

            startPoseY = FieldCoordinates.RED_HIVE_CENTER_LINE.y + (ROBOT_LENGTH_INCHES / 2);

            startHeadingDegrees = FieldCoordinates.TOWARDS_AUDIENCE;

            launchPoseY = startPoseY + 12;

            midpointCurveY = 34;

            midpointCurveX = 24;

            endPoseX = FieldCoordinates.LOADING_ZONE_RED_YCENTER_AGAINST_THE_WALL.x + PARK_DISTANCE_OFF_WALL;

            endPoseY = FieldCoordinates.LOADING_ZONE_RED_YCENTER_AGAINST_THE_WALL.y - 13;

            endHeadingDegrees = FieldCoordinates.TOWARDS_AUDIENCE;
        } else {
            startPoseX = FieldCoordinates.BLUE_HIVE_CENTER_LINE.x;

            startPoseY = 144 - (ROBOT_LENGTH_INCHES / 2);

            startHeadingDegrees = FieldCoordinates.AWAY_FROM_AUDIENCE;

            launchPoseY = startPoseY - 12;

            midpointCurveY = 144-34;

            midpointCurveX = 144-24;

            endPoseX = FieldCoordinates.LOADING_ZONE_BLUE_YCENTER_AGAINST_THE_WALL.x - PARK_DISTANCE_OFF_WALL;

            endPoseY = FieldCoordinates.LOADING_ZONE_BLUE_YCENTER_AGAINST_THE_WALL.y + 13;

            endHeadingDegrees = FieldCoordinates.AWAY_FROM_AUDIENCE;
        }

        final Pose startPose = poseOfDegrees(startPoseX, startPoseY, startHeadingDegrees); // Start Pose of our robot.

        final Pose launchPose = poseOfDegrees(startPoseX, launchPoseY, startHeadingDegrees);

        final Pose endPose = poseOfDegrees(endPoseX, endPoseY, endHeadingDegrees);

        final Path toLaunchPath = line(startPose, launchPose);
        preWarmLauncer(sequenceOfStates);

        linear(toLaunchPath, startPose, launchPose);
        PedroFollowerState toLaunchPathState =
                new PedroFollowerState("to and launch", telemetry, pedroFollower, toLaunchPath);
        sequenceOfStates.addSequential(toLaunchPathState);

        addLaunchSteps(sequenceOfStates);

        final Pose midpointPose = poseOfDegrees(midpointCurveX,midpointCurveY,startHeadingDegrees);

        final Path parkPath = curve(List.of(launchPose, midpointPose, endPose));
        linear(parkPath, startPose, endPose);

        pedroFollower.setPose(startPose);

        PedroFollowerState parkPathState = new PedroFollowerState("leave and park", telemetry, pedroFollower, parkPath);

        sequenceOfStates.addSequential(parkPathState);

        commonCompletionStates(sequenceOfStates);

        stateMachine.addSequence(sequenceOfStates);
    }

    private void preWarmLauncer(SequenceOfStates sequenceOfStates) {
        final State preWarmLauncher = new RunnableState("prewarm launcher", telemetry,
                () -> wheeledLauncher.launchVelocityAuto());
        sequenceOfStates.addSequential(preWarmLauncher);
    }

    @androidx.annotation.NonNull
    private static Path line(Pose startPose, Pose endPose) {
        return new Path(new BezierLine(startPose, endPose));
    }

    private static Path curve(List<Pose> poses) {
        return new Path(new BezierCurve(poses));
    }

    private void addLaunchSteps(final SequenceOfStates sequenceOfStates) {
        final State startIntake = new RunnableState("startIntake", telemetry,
                () -> intake.intakeAuto());

        sequenceOfStates.addSequential(startIntake);
        wheeledLauncher.addLaunchStates(sequenceOfStates);

        final State stopIntake = new RunnableState("stopIntake", telemetry,
                () -> intake.stop());

        sequenceOfStates.addSequential(stopIntake);
    }

    private Pose poseOfDegrees(final double x, final double y, final double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }
}