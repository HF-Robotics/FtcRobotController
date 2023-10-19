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

package com.hfrobots.tnt.season2223;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import androidx.annotation.NonNull;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.google.common.base.Optional;
import com.google.common.base.Supplier;
import com.google.common.base.Ticker;
import com.google.common.collect.Sets;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.drive.mecanum.MultipleTrajectoriesFollowerState;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveBase;
import com.hfrobots.tnt.corelib.drive.mecanum.TurnState;
import com.hfrobots.tnt.corelib.drive.mecanum.util.AxisDirection;
import com.hfrobots.tnt.corelib.state.ReadyCheckable;
import com.hfrobots.tnt.season2223.pipelines.ColorSignalDetectorPipeline;
import com.hfrobots.tnt.season2223.pipelines.CyanGripPipeline;
import com.hfrobots.tnt.season2223.pipelines.GreenGripPipeline;
import com.hfrobots.tnt.season2223.pipelines.PinkGripPipeline;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Set;
import java.util.concurrent.TimeUnit;

@Autonomous(name="00 PP Auto")
@Disabled
public class PowerPlayAuto extends OpMode {
    private Ticker ticker;

    private RoadRunnerMecanumDriveBase driveBase;

    private StateMachine stateMachine;

    // Used to ensure that states with circular dependencies are setup correctly
    private final Set<ReadyCheckable> readyCheckables = Sets.newHashSet();

    private SignalDetector signalDetector;

    private LiftMechanism liftMechanism;

    private Gripper gripper;

    private OperatorControls operatorControls;

    private FakeOnOffButton goSmallJunctionAutoButton = new FakeOnOffButton();

    private FakeOnOffButton liftToBottom = new FakeOnOffButton();
    private PowerPlayDriveConstants driveConstants;

    private enum LOCATION {
        LOCATION_ONE,
        LOCATION_TWO,
        LOCATION_THREE;
    }

    private LOCATION detectedLocation = LOCATION.LOCATION_THREE;

    // The routes our robot knows how to do - rename these to something meaningful for the season!
    private enum Routes {
        DETECT_SIGNAL("Detect Signal");
        //SCORE_PRELOAD("Score Preload");

        final String description;

        Routes(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    private int selectedRoutesIndex = 0;

    private Routes[] possibleRoutes = Routes.values();

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Constants.Alliance currentAlliance = Constants.Alliance.RED;

    private int initialDelaySeconds = 0;

    private PowerPlayDriveTeamSignal driveTeamSignal;

    // private Rev2mDistanceSensor leftSideDistanceSensor = null;

    // private Rev2mDistanceSensor rightSideDistanceSensor = null;

    @Override
    public void init() {
        ticker = createAndroidTicker();

        setupDriverControls();

        setupLift();

        setupOperatorControls();

        driveConstants = new PowerPlayDriveConstants();
        driveBase = new RoadRunnerMecanumDriveBase(hardwareMap,
                driveConstants);

        stateMachine = new StateMachine(telemetry);

        driveTeamSignal = new PowerPlayDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

//        try {
//            leftSideDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
//            rightSideDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");;
//        } catch (Exception ex) {
//            // Don't attempt to use either
//            leftSideDistanceSensor = null;
//            rightSideDistanceSensor = null;
//        }

        setupOpenCvCameraAndPipeline();
    }

    private void setupOperatorControls() {
        NinjaGamePad operatorsGamepad = new NinjaGamePad(gamepad2);

        operatorControls = OperatorControls.builder()
                .operatorGamepad(operatorsGamepad)
                .liftMechanism(liftMechanism)
                .gripper(gripper).build();
    }

    private void setupLift() {
        Servo gripperServo = hardwareMap.get(Servo.class, "gripperServo");

        gripper = new Gripper(gripperServo);

        liftMechanism = LiftMechanism.builderFromHardwareMap(hardwareMap, telemetry)
                .gripper(gripper).build();
    }

    private com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera pipelineAndCamera;

    private void setupOpenCvCameraAndPipeline() {
        // FIXME: Auto really should not know these cyan/pink/whatever details
        CyanGripPipeline cyanContourFinder = new CyanGripPipeline();
        PinkGripPipeline pinkContourFinder = new PinkGripPipeline();
        GreenGripPipeline greenContourFinder = new GreenGripPipeline();
        ColorSignalDetectorPipeline pipeline = new ColorSignalDetectorPipeline(
                cyanContourFinder,
                pinkContourFinder, greenContourFinder, telemetry);

        signalDetector = pipeline;

        com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.EasyOpenCvPipelineAndCameraBuilder pipelineBuilder =
                com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.builder();

        pipelineBuilder.hardwareMap(hardwareMap).telemetry(telemetry)
                .openCvPipeline(pipeline);

        pipelineAndCamera = pipelineBuilder.build();

        pipelineAndCamera.createAndRunPipeline();
    }

    @Override
    public void start() {
        super.start();

        // Gives control to move the lift to the state machine
        liftMechanism.setLiftGoSmallButton(goSmallJunctionAutoButton.debounced());
        liftMechanism.setLiftLowerLimitButton(liftToBottom.debounced());

        if (pipelineAndCamera != null) {
            try {
                pipelineAndCamera.pauseViewport();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Error while pausing viewport, continuing...", t);
            }
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    private boolean configLocked = false;

    @Override
    public void init_loop() {
        if (driversGamepad == null) { // safety, need to double check whether we actually need this
            // not ready yet init() hasn't been called
            return;
        }

        if (!configLocked) {
            doAutoConfig();

            if (lockButton.getRise()) {
                configLocked = true;
            }
        } else {
            if (unlockButton.getRise()) {
                configLocked = false;
            }
        }

        if (configLocked) {
            telemetry.addData("00", "LOCKED: Press Rt stick unlock");
        } else {
            telemetry.addData("00", "UNLOCKED: Press Lt stick lock");
        }

        if (operatorControls != null) {
            operatorControls.periodicTask();
        }

        driveTeamSignal.setAlliance(currentAlliance);
        driveTeamSignal.periodicTask();

        telemetry.addData("01", "Alliance: %s", currentAlliance);
        telemetry.addData("02", "Task: %s", possibleRoutes[selectedRoutesIndex].getDescription());
        telemetry.addData("03", "Delay %d sec", initialDelaySeconds);

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
        // Use driver dpad up/down to select which route to run
        if (driverDpadDown.getRise()) {
            selectedRoutesIndex--;

            if (selectedRoutesIndex < 0) {
                selectedRoutesIndex = possibleRoutes.length - 1; // why?
            }
        } else if (driverDpadUp.getRise()) {
            selectedRoutesIndex++;

            if (selectedRoutesIndex > possibleRoutes.length - 1) { // why -1?
                selectedRoutesIndex = 0;
            }
        }

        // use left/right bumper to decrease/increase delay

        if (driverLeftBumper.getRise()) {
            initialDelaySeconds -= 1;

            if (initialDelaySeconds < 0) {
                initialDelaySeconds = 0;
            }
        } else if (driverRightBumper.getRise()) {
            initialDelaySeconds += 1;

            if (initialDelaySeconds > 25) {
                initialDelaySeconds = 25;
            }
        }

        // Alliance selection
        if (driverBRedButton.getRise()) {
            currentAlliance = Constants.Alliance.RED;
        } else if (driverXBlueButton.getRise()) {
            currentAlliance = Constants.Alliance.BLUE;
        }
    }

    private boolean stateMachineSetup = false;

    @Override
    public void loop() {
        try {
            if (!stateMachineSetup) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

                switch (selectedRoute) {
                    case DETECT_SIGNAL:
                        setupRouteChoiceDetected();
                        break;
                    //case SCORE_PRELOAD:
                        //setupRouteChoiceDetectAndScorePreload();
                      //  break;
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

            liftMechanism.periodicTask();

            driveTeamSignal.periodicTask();

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

    protected void setupRouteChoiceDetected() {

        final State detectState = createSignalDetectorState();

        final State moveRobot = new MultipleTrajectoriesFollowerState("Move robot",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                switch (detectedLocation) {
                    case LOCATION_ONE: {
                        addTrajectoryProvider("Off wall", (t) -> t.forward(3));
                        addTrajectoryProvider("Align with location", (t) -> t.strafeLeft(29 - 3));
                        addTrajectoryProvider("Into location", (t) -> t.forward(24 + 15));
                        break;
                    }
                    case LOCATION_TWO: {
                        // needs to be even further!!!
                        addTrajectoryProvider("Off wall 1", (t) -> t.forward(12));
                        addTrajectoryProvider("Off wall 2", (t) -> t.forward(12 + 4));
                        addTrajectoryProvider("Push signal", (t) -> t.forward(4 + 4));
                        addTrajectoryProvider("Retreat from signal", (t) -> t.back(8));
                        break;
                    }
                    case LOCATION_THREE: {
                        addTrajectoryProvider("Off wall", (t) -> t.forward(3));
                        addTrajectoryProvider("Align with location", (t) -> t.strafeRight(29 - 3));
                        addTrajectoryProvider("Into location", (t) -> t.forward(24 + 15));
                        break;
                    }
                }
            }
        };

        final SequenceOfStates sequence = new SequenceOfStates(ticker, telemetry);
        sequence.addSequential(detectState);

        sequence.addSequential(new RunnableState("Ensure cone gripped", telemetry,
                gripper::close));

        sequence.addWaitStep("Wait for gripper", 500, TimeUnit.MILLISECONDS);

        sequence.addSequential(new RunnableState("Lift cone", telemetry,
                () -> goSmallJunctionAutoButton.setPressed(true)));

        sequence.addWaitStep("wait on cone", 1, TimeUnit.SECONDS);

        sequence.addSequential(new RunnableState("Clear lift cone", telemetry,
                () -> goSmallJunctionAutoButton.setPressed(false)));

        sequence.addSequential(moveRobot);

        sequence.addSequential(new RunnableState("Move cone down", telemetry,
                () -> liftToBottom.setPressed(true)));

        sequence.addSequential(newDoneState("Done!"));
        stateMachine.addSequence(sequence);
    }

    @NonNull
    private State createSignalDetectorState() {
        State detectState = new StopwatchTimeoutSafetyState("Detect Signal",
                telemetry, ticker, 5000) {
            @Override
            public State doStuffAndGetNextState() {
                if (isTimedOut()) {
                    resetTimer();

                    return nextState;
                }

                SignalDetector.DetectedSignal signal = signalDetector.getDetectedSignal();

                switch (signal) {
                    case ORIENTATION_A: {
                        detectedLocation = LOCATION.LOCATION_ONE;
                        resetTimer();

                        return nextState;
                    }
                    case ORIENTATION_B: {
                        detectedLocation = LOCATION.LOCATION_TWO;
                        resetTimer();

                        return nextState;
                    }
                    case ORIENTATION_C: {
                        detectedLocation = LOCATION.LOCATION_THREE;
                        resetTimer();

                        return nextState;
                    }
                }

                return this;
            }
        };
        return detectState;
    }

    enum Side {
        LEFT, RIGHT, UNKNOWN;
    }

//    protected void setupRouteChoiceDetectAndScorePreload() {
//        // Failsafe - run auto that doesn't require distance sensors
//        if (leftSideDistanceSensor == null || rightSideDistanceSensor == null) {
//            Log.e(LOG_TAG, "Missing or broken distance sensor, running navigation only auto");
//
//            setupRouteChoiceDetected();
//
//            return;
//        }
//
//        Side sideOfField = Side.UNKNOWN;
//        double distanceLeftIn = 0;
//        double distanceRightIn = 0;
//
//        try {
//            distanceLeftIn = leftSideDistanceSensor.getDistance(DistanceUnit.INCH);
//            distanceRightIn = rightSideDistanceSensor.getDistance(DistanceUnit.INCH);
//
//            if (distanceLeftIn < distanceRightIn) {
//                if (distanceLeftIn > 36 || distanceLeftIn < 21) {
//                    sideOfField = Side.UNKNOWN;
//                } else {
//                    sideOfField = Side.LEFT;
//                }
//            } else if (distanceLeftIn > distanceRightIn) {
//                if (distanceRightIn > 36 || distanceRightIn < 21) {
//                    sideOfField = Side.UNKNOWN;
//                } else {
//                    sideOfField = Side.RIGHT;
//                }
//            }
//        } catch (Throwable t) {
//            Log.e(LOG_TAG, "Distance sensors failed, using UNKNOWN side for setup", t);
//        }
//
//        if (sideOfField == Side.UNKNOWN) {
//            Log.e(LOG_TAG, "Bad distances, sensor failure or bad setup, (L): "
//                    + distanceLeftIn + ", (R): " + distanceRightIn);
//            setupRouteChoiceDetected();
//
//            return;
//        }
//
//        // This is the same as navigation-only auto, later improvement
//        // to extract common method?
//
//        final State detectState = createSignalDetectorState();
//
//        final Side detectedSideOfField = sideOfField;
//
//        final SequenceOfStates sequence = new SequenceOfStates(ticker, telemetry);
//        sequence.addSequential(detectState);
//
//        sequence.addSequential(new RunnableState("Ensure cone gripped", telemetry,
//                gripper::close));
//
//        sequence.addWaitStep("Wait for gripper", 500, TimeUnit.MILLISECONDS);
//
//        sequence.addSequential(new RunnableState("Lift cone", telemetry,
//                () -> goSmallJunctionAutoButton.setPressed(true)));
//
//        sequence.addWaitStep("wait on cone", 1, TimeUnit.SECONDS);
//
//        sequence.addSequential(new RunnableState("Clear lift cone", telemetry,
//                () -> goSmallJunctionAutoButton.setPressed(false)));
//
//        // States to program:
//        //
//        // (1) Move the robot to the junction
//        //
//        // (2) Drop the cone
//        //
//        // (3) Wait a bit before moving again
//        //
//        // (4) Move the robot to the detected signal parking zone
//
//        // Example of how to move the robot
//        // (1) Move Robot to Junction
//        final State moveRobotToJunction = new MultipleTrajectoriesFollowerState("Move robot to junction",
//                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
//            @Override
//            protected void createTrajectoryProviders() {
//                addTrajectoryProvider("Off wall", t -> t.forward(3));
//
//                if (detectedSideOfField == Side.LEFT) {
//                    addTrajectoryProvider("Align with location", t -> t.strafeRight(14.5));
//                } else { addTrajectoryProvider("Align with location", t -> t.strafeLeft(12.875));
//                    // Right side
//                }
//                addTrajectoryProvider( "Forward to Junction", t -> t.forward(8.75-3-2));
//                addTrajectoryProvider("Scooch 1", t -> t.forward(2));
//                addTrajectoryProvider("Scooch 2", t -> t.forward(2));
//                addTrajectoryProvider("Scooch 3", t -> t.forward(2));
//                addTrajectoryProvider("Scooch 4", t -> t.back(4));
//            }
//        };
//
//        sequence.addSequential(moveRobotToJunction);
//
//        sequence.addSequential(new RunnableState("Cone aligns to junction", telemetry,
//                () -> liftMechanism.liftToScorePreloadPosition()));
//
//        sequence.addWaitStep("Wait for lift to move down", 750, TimeUnit.MILLISECONDS);
//
//        sequence.addSequential(new RunnableState("mic drop", telemetry,
//                gripper::open));
//
//        sequence.addWaitStep("Wait for gripper", 500, TimeUnit.MILLISECONDS);
//
//        // FIXME - remove and replace with code written by team - MM
//        sequence.addSequential(new RunnableState("Lift to clear junction", telemetry,
//                () -> goSmallJunctionAutoButton.setPressed(true)));
//
//        sequence.addWaitStep("wait on lift", 1, TimeUnit.SECONDS);
//
//        sequence.addSequential(new RunnableState("Unpress button", telemetry,
//                () -> goSmallJunctionAutoButton.setPressed(false)));
//
//        final State moveRobotBackFromJunction = new MultipleTrajectoriesFollowerState(
//                "Move robot back from junction",
//                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
//            @Override
//            protected void createTrajectoryProviders() {
//                addTrajectoryProvider("Load up wheels", (t) -> t.forward(1));
//                addTrajectoryProvider("back it up", (t) -> t.back(3));
//            }
//        };
//
//        sequence.addSequential(moveRobotBackFromJunction);
//
//        sequence.addWaitStep("wait for momentum to settle", 500, TimeUnit.MILLISECONDS);
//
//        final State moveRobotToCorrectZone = new MultipleTrajectoriesFollowerState(
//                "Move robot to detected zone",
//                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
//            @Override
//            protected void createTrajectoryProviders() {
//                switch (detectedLocation) {
//                    case LOCATION_ONE: {
//                        addTrajectoryProvider("Align with location", (t) -> t.strafeLeft(26));
//                        addTrajectoryProvider("Into location", (t) -> t.forward(24 + 15));
//                        break;
//                    }
//                    case LOCATION_TWO: {
//                        // needs to be even further!!!
//                        if (detectedSideOfField == Side.LEFT) {
//                            addTrajectoryProvider("Align with Location", (t) -> t.strafeLeft(6.5));
//                        } else {
//                            addTrajectoryProvider("Align with location", (t) -> t.strafeRight(6.5));
//                        }
//
//                        addTrajectoryProvider("Off wall 1", (t) -> t.forward(12));
//                        addTrajectoryProvider("Off wall 2", (t) -> t.forward(12 + 4));
//                        addTrajectoryProvider("Push signal", (t) -> t.forward(4 + 4));
//                        addTrajectoryProvider("Retreat from signal", (t) -> t.back(8));
//                        break;
//                    }
//                    case LOCATION_THREE: {
//                        addTrajectoryProvider("Align with location", (t) -> t.strafeRight(11.75));
//                        addTrajectoryProvider("Into location", (t) -> t.forward(24 + 15));
//                        break;
//                    }
//                }
//            }
//        };
//
//        sequence.addSequential(moveRobotToCorrectZone);
//
//        // This is common end to auto
//
//        sequence.addSequential(new RunnableState("Move lift down", telemetry,
//                () -> liftToBottom.setPressed(true)));
//
//        sequence.addSequential(newDoneState("Done!"));
//        stateMachine.addSequence(sequence);
//    }

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

    // TODO: Move to new model of controllers
    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected RangeInput driverRightStickX;

    protected RangeInput driverRightStickY;

    protected DebouncedButton driverDpadUp;

    protected DebouncedButton driverDpadDown;

    protected DebouncedButton driverDpadLeft;

    protected DebouncedButton driverDpadRight;

    protected DebouncedButton driverXBlueButton;

    protected DebouncedButton driverBRedButton;

    protected DebouncedButton driverYYellowButton;

    protected DebouncedButton driverAGreenButton;

    protected DebouncedButton driverRightBumper;

    protected DebouncedButton driverLeftBumper;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected NinjaGamePad driversGamepad;

    private void setupDriverControls() {
        driversGamepad = new NinjaGamePad(gamepad1);
        driverLeftStickX = driversGamepad.getLeftStickX();
        driverLeftStickY = driversGamepad.getLeftStickY();
        driverRightStickX = driversGamepad.getRightStickX();
        driverRightStickY = driversGamepad.getRightStickY();

        driverDpadDown = new DebouncedButton(driversGamepad.getDpadDown());
        driverDpadUp = new DebouncedButton(driversGamepad.getDpadUp());
        driverDpadLeft = new DebouncedButton(driversGamepad.getDpadLeft());
        driverDpadRight = new DebouncedButton(driversGamepad.getDpadRight());
        driverAGreenButton = new DebouncedButton(driversGamepad.getAButton());
        driverBRedButton = new DebouncedButton(driversGamepad.getBButton());
        driverXBlueButton = new DebouncedButton(driversGamepad.getXButton());
        driverYYellowButton = new DebouncedButton(driversGamepad.getYButton());
        driverLeftBumper = new DebouncedButton(driversGamepad.getLeftBumper());
        driverRightBumper = new DebouncedButton(driversGamepad.getRightBumper());
        lockButton = new DebouncedButton(driversGamepad.getLeftStickButton());
        unlockButton = new DebouncedButton(driversGamepad.getRightStickButton());
    }
}