/*
 Copyright (c) 2023 The Tech Ninja Team (https://ftc9929.com)

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
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Supplier;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.drive.mecanum.DriveConstants;
import com.hfrobots.tnt.corelib.drive.mecanum.MultipleTrajectoriesFollowerState;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveBase;
import com.hfrobots.tnt.corelib.drive.mecanum.TurnState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Autonomous(name="00 CS Auto")
public class CenterstageAuto extends OpMode {

    public static final int SPLINE_RIGHT_SIGN = -1;

    public static final int SPLINE_LEFT_SIGN = 1;

    private CenterstageDriveTeamSignal driveTeamSignal;

    private Hanger hanger;

    private ScoringMechanism scoringMechanism;

    interface InitLoopConfigTask {
        void chooseBlueAlliance();

        void chooseRedAlliance();

        void nextTaskChoice();

        void previousTaskChoice();

        void increaseDelay();

        void decreaseDelay();

        void lockConfig();

        void unlockConfig();
    }

    private enum SpikeStripLocation {
        LOCATION_LEFT,
        LOCATION_CENTER,
        LOCATION_RIGHT;
    }

    private SpikeStripLocation detectedLocation = SpikeStripLocation.LOCATION_CENTER;

    private Ticker ticker;

    private RoadRunnerMecanumDriveBase driveBase;

    private StateMachine stateMachine;

    private CenterstageOperatorControls operatorControls;

    private CenterstageDriverControls driverControls;

    private DriveConstants driveConstants;

    private VisionPortal visionPortal;

    private final SpikeStripDetector spikeStripDetector = new SpikeStripDetector();

    private Intake intake;

    // The tasks our robot knows how to do - rename these to something meaningful for the season!
    private enum TaskChoice {
        BACKSTAGE_SIDE("Start from backstage side"),
        WING_SIDE("Start from wing side");

        final String description;

        TaskChoice(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    private int selectedTaskIndex = 0;

    private TaskChoice[] possibleTaskChoices = TaskChoice.values();

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Constants.Alliance currentAlliance = Constants.Alliance.RED;

    private int initialDelaySeconds = 0;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            intake = new Intake(hardwareMap);

            hanger = new Hanger(hardwareMap);

            scoringMechanism = ScoringMechanism.builderFromHardwareMap(hardwareMap, telemetry).build();

            driveTeamSignal = new CenterstageDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

            setupDriverControls();
            setupOperatorControls();
            setupVisionPortal(hardwareMap);

            // Safety, in case we change the defaults
            if (currentAlliance == Constants.Alliance.RED) {
                spikeStripDetector.useRedPipeline();
            } else {
                spikeStripDetector.useBluePipeline();
            }

            driveConstants = new CenterstageDriveConstants();

            driveBase = new RoadRunnerMecanumDriveBase(hardwareMap,
                    driveConstants);

            stateMachine = new StateMachine(telemetry);
        });
    }

    private void setupDriverControls() {
        NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

        driverControls = CenterstageDriverControls.builder()
                .driversGamepad(driversGamepad).autoConfigTask(new InitLoopConfigTask() {
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
                            spikeStripDetector.useBluePipeline();
                        }
                    }

                    @Override
                    public void chooseRedAlliance() {
                        if (!configLocked) {
                            currentAlliance = Constants.Alliance.RED;
                            spikeStripDetector.useRedPipeline();
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
        NinjaGamePad operatorsGamepad = new NinjaGamePad(gamepad2);

        operatorControls = CenterstageOperatorControls.builder()
                .operatorGamepad(operatorsGamepad)
                .intake(intake)
                .scoringMechanism(scoringMechanism)
                .hanger(hanger)
                .driveTeamSignal(driveTeamSignal).build();
    }

    private void setupVisionPortal(final HardwareMap hardwareMap) {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        builder.addProcessor(spikeStripDetector);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    @Override
    public void start() {
        Shared.withBetterErrorHandling(() -> {
            super.start();
            spikeStripDetector.recordDetections();
            visionPortal.stopLiveView();
        });
    }

    @Override
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

            if (currentAlliance == Constants.Alliance.BLUE) {
                spikeStripDetector.useBluePipeline();
            } else {
                spikeStripDetector.useRedPipeline();
            }

            driveTeamSignal.setAlliance(currentAlliance);
            driveTeamSignal.periodicTask();

            updateTelemetry(telemetry);
        });
    }

    private Ticker createAndroidTicker() {
        return new Ticker() {
            public long read() {
                return android.os.SystemClock.elapsedRealtimeNanos();
            }
        };
    }

    private boolean stateMachineSetup = false;

    @Override
    public void loop() {
        Shared.withBetterErrorHandling(() -> {
            if (!stateMachineSetup) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                TaskChoice selectedTaskChoice = possibleTaskChoices[selectedTaskIndex];

                switch (selectedTaskChoice) {
                    case WING_SIDE:
                        setupStartWingSideStateMachine();
                        break;
                    case BACKSTAGE_SIDE:
                        setupStartBackstageSideStateMachine();
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

            telemetry.update(); // send all telemetry to the drivers' station
        });
    }

    protected void setupStartWingSideStateMachine() {
        sharedStateMachineSetup(false);
    }

    protected void setupStartBackstageSideStateMachine() {
        sharedStateMachineSetup(true);
    }

    protected void sharedStateMachineSetup(final boolean isBackstage) {

        final State detectState = createSpikeStripDetectorState();

        //
        // FIXME - Remove this once detector is working
        //

        detectedLocation = SpikeStripLocation.LOCATION_LEFT;
        currentAlliance = Constants.Alliance.BLUE;

        //
        // End test code before detector working
        //

        final State moveRobotToSpikeStrips = new MultipleTrajectoriesFollowerState("Move robot",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                switch (detectedLocation) {
                    case LOCATION_LEFT: {
                        addTrajectoryProvider("Off wall", (t) -> t.forward(27));

                        break;
                    }
                    case LOCATION_CENTER: {
                        // It's quicker to spline, but remember x is fwd/back, y is left/right

                        final double yScale = 3;
                        double ySign = SPLINE_RIGHT_SIGN;

                        if (isBackstage) {
                            if (currentAlliance == Constants.Alliance.RED) {
                                // RED -> Right (-y)
                                ySign = SPLINE_RIGHT_SIGN;
                            } else {
                                // BLUE -> LEFT (+y)
                                ySign = SPLINE_LEFT_SIGN;
                            }
                        } else {
                            if (currentAlliance == Constants.Alliance.RED) {
                                // RED -> Left (+y)
                                ySign = SPLINE_LEFT_SIGN;
                            } else {
                                // BLUE -> RIGHT (-y)
                                ySign = SPLINE_RIGHT_SIGN;
                            }
                        }

                        final double yDelta = yScale * ySign;

                        addTrajectoryProvider("Off wall", (t) -> t.splineToLinearHeading(new Pose2d(27D, yDelta, 0),0));

                        break;
                    }
                    case LOCATION_RIGHT: {
                        addTrajectoryProvider("Off wall", (t) -> t.forward(27));

                        break;
                    }
                }
            }
        };

        final State turnRobot = new TurnState("Turn towards spike strip",
                telemetry,
                (Supplier<Turn>) () -> {
                    switch (detectedLocation) {
                        case LOCATION_LEFT: {
                            // this turn does not depend on starting location or alliance
                            return new Turn(Rotation.CCW, 90);
                        }
                        case LOCATION_CENTER: {
                            // We don't need to turn
                            return null;
                        }
                        case LOCATION_RIGHT: {
                            // this turn does not depend on starting location or alliance
                            return new Turn(Rotation.CW, 90);
                        }
                    }

                    return null;
                }, driveBase,ticker,30_000);

        final State moveRobotToBackstage = new MultipleTrajectoriesFollowerState("Move robot to backstage",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                switch (detectedLocation) {
                    case LOCATION_LEFT: {
                        addTrajectoryProvider("Fwd a bit", (t) ->t.forward(1));
                        addTrajectoryProvider("Away from spike strip", (t) ->t.strafeLeft(22));
                        addTrajectoryProvider("Fwd a bit", (t) ->t.forward(1));
                        addTrajectoryProvider("To backstage", (t) -> t.forward(47));

                        break;
                    }
                    case LOCATION_CENTER: {
                        addTrajectoryProvider("Fwd a bit", (t) -> t.forward(3));
                        addTrajectoryProvider("Away from spike strip", (t) -> t.back(26));
                        addTrajectoryProvider("To backstage", (t) -> t.strafeLeft(47));

                        break;
                    }
                    case LOCATION_RIGHT: {
                        //addTrajectoryProvider("Direct to backstage", (t) -> t.forward(1).back(50));
                        addTrajectoryProvider("Off wall", (t) -> t.splineToLinearHeading(new Pose2d(-50, -3D, 0),0));

                        break;
                    }
                }
            }
        };

        final SequenceOfStates sequence = new SequenceOfStates(ticker, telemetry);

        sequence.addSequential(detectState);
        sequence.addSequential(moveRobotToSpikeStrips);
        sequence.addSequential(turnRobot);

        // Outtake the pixel
        sequence.addSequential(new RunnableState("start outtake", telemetry, () -> {
            if (intake != null) {
                intake.out(1);
            }
        }));

        sequence.addSequential(newDelayState("Place pixel", 1, TimeUnit.SECONDS));

        // Outtake the pixel
        sequence.addSequential(new RunnableState("stop outtake", telemetry, () -> {
            if (intake != null) {
                intake.stop();
            }
        }));

        if (isBackstage) {
            // FIXME: Simplify for now, some of these trajectories are busted
            //sequence.addSequential(moveRobotToBackstage);
        }

        sequence.addSequential(newDoneState("Done!"));
        stateMachine.addSequence(sequence);
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
                    driveBase.setMotorPowers(0, 0, 0, 0);

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

    public static class CenterstageDriveConstants extends DriveConstants {

        private static DriveConstraints DRIVE_CONSTRAINTS = new DriveConstraints(80, 30.0, 0.0,
                Math.toRadians(46.5), Math.toRadians(46.5), 0.0);

        private static PIDCoefficients TRANSLATIONAL_PID_COEFFICIENTS = new PIDCoefficients(4.2D, 0, 0);

        private static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0.295D, 0, 0);

        private static double ENCODER_PPR = 384.5;

        private static double WHEEL_RADIUS_IN = 1.89;

        private static double TRACK_WIDTH_IN = 9.0;

        private static double MAX_MOTOR_RPM = 435;

        private static double GEAR_RATIO = 1.0;

        @Override
        public DriveConstraints getBaseConstraints() {
            return DRIVE_CONSTRAINTS;
        }

        @Override
        public PIDCoefficients getTranslationalPID() {
            return TRANSLATIONAL_PID_COEFFICIENTS;
        }

        @Override
        public PIDCoefficients getHeadingPid() {
            return HEADING_PID_COEFFICIENTS;
        }

        @Override
        public DriveConstraints getDriveConstraints() {
            return DRIVE_CONSTRAINTS;
        }

        @Override
        public double encoderTicksToInches(double ticks) {
            return WHEEL_RADIUS_IN * 2 * Math.PI * GEAR_RATIO * ticks / ENCODER_PPR;
        }

        @Override
        public double rpmToVelocity(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS_IN / 60.0;
        }

        @Override
        public double getMaxRpm() {
            return MAX_MOTOR_RPM;
        }

        @Override
        public double getTrackWidth() {
            return TRACK_WIDTH_IN;
        }
    }

    @NonNull
    private State createSpikeStripDetectorState() {
        State detectState = new StopwatchTimeoutSafetyState("Detect Spike Strip",
                telemetry, ticker, 5000) {
            @Override
            public State doStuffAndGetNextState() {
                if (isTimedOut()) {
                    Log.e(LOG_TAG, "Timed out waiting for detection, using " +
                            detectedLocation + " as location");

                    return transitionToNextState();
                }

                SpikeStripDetector.DetectedSpikeStrip detectedSpikeStrip =
                        spikeStripDetector.getDetectedSpikeStrip();

                switch (detectedSpikeStrip) {
                    case LEFT: {
                        detectedLocation = SpikeStripLocation.LOCATION_LEFT;
                        resetTimer();

                        Log.i(LOG_TAG, "Detected LEFT location");

                        return transitionToNextState();
                    }
                    case CENTER: {
                        detectedLocation = SpikeStripLocation.LOCATION_CENTER;
                        resetTimer();

                        Log.i(LOG_TAG, "Detected CENTER location");

                        return transitionToNextState();
                    }
                    case RIGHT: {
                        detectedLocation = SpikeStripLocation.LOCATION_RIGHT;
                        resetTimer();

                        Log.i(LOG_TAG, "Detected RIGHT location");

                        return transitionToNextState();
                    }
                }

                return this;
            }

            private State transitionToNextState() {
                resetTimer();

                try {
                    visionPortal.stopStreaming();
                } catch (Throwable t) {
                    Log.e(LOG_TAG, "Caught exception while stopping vision portal, ignoring...", t);
                }

                return nextState;
            }
        };

        return detectState;
    }

    private State newDelayState(final String name, long amount, TimeUnit units) {
        return new StopwatchDelayState(name, telemetry, ticker, amount, units);
    }
}