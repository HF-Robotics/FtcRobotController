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

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Preconditions;
import com.google.common.base.Ticker;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveBase;
import com.hfrobots.tnt.corelib.drive.mecanum.TrajectoryAwareSequenceOfStates;
import com.hfrobots.tnt.corelib.drive.mecanum.TrajectoryFollowerState;
import com.hfrobots.tnt.corelib.state.ReadyCheckable;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import lombok.Setter;

@Autonomous(name="00 Frt Frnzy Auto")
public class Auto extends OpMode {
    public static final int TOO_FAR_TO_DRIVE_INTO_STORAGE = 4;
    public static final int PERFECT_WALL_STORAGE_DISTANCE = 29;
    private Ticker ticker;

    private OnOffButton unsafe = new OnOffButton() {
        @Override
        public boolean isPressed() {
            return true;
            //Danger Will Robinson
        }

        @Override
        public DebouncedButton debounced() {
            return null;
        }
    };

    private RoadRunnerMecanumDriveBase driveBase;

    private StateMachine stateMachine;

    private CarouselMechanism carouselMechanism;

    private FreightManipulator freightManipulator;

    private BarcodeDetectorPipeline barcodeDetectorPipeline;

    private BarcodeDetectorPipeline.BarcodePosition detectedPosition = BarcodeDetectorPipeline.BarcodePosition.UNKNOWN;

    private DriveTeamSignal driveTeamSignal;

    private final Set<ReadyCheckable> readyCheckables = Sets.newHashSet();

    // The routes our robot knows how to do
    private enum Routes {
        DETECT_BARCODE_DUCK_STORAGE("Barcode - duck - storage"),
        DELIVER_DUCK_PARK_STORAGE("Del. duck - park storage"),
        PARK_WAREHOUSE("Park warehouse");

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

    private OperatorControls operatorControls;

    private DistanceSensor rearDistanceSensor;

    @Override
    public void init() {
        ticker = createAndroidTicker();

        setupDriverControls();

        rearDistanceSensor = hardwareMap.get(DistanceSensor.class, "rearDistanceSensor");

        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);

        driveBase = new RoadRunnerMecanumDriveBase(hardwareMap,
                new FreightFrenzyDriveConstants());

        stateMachine = new StateMachine(telemetry);

        driveTeamSignal = new DriveTeamSignal(hardwareMap, ticker);

        setupOpenCvCameraAndPipeline();

        carouselMechanism = new CarouselMechanism(hardwareMap);

        freightManipulator = new FreightManipulator(hardwareMap, telemetry, ticker);

        operatorControls = OperatorControls.builder().carouselMechanism(carouselMechanism)
                .freightManipulator(freightManipulator)
                .operatorGamepad(new NinjaGamePad(gamepad2)).build();
    }

    private com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera pipelineAndCamera;

    private void setupOpenCvCameraAndPipeline() {

        // Create the pipeline
        barcodeDetectorPipeline = new BarcodeDetectorPipeline(telemetry, false);

        com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.EasyOpenCvPipelineAndCameraBuilder pipelineBuilder =
                com.hfrobots.tnt.corelib.vision.EasyOpenCvPipelineAndCamera.builder();

        pipelineBuilder.hardwareMap(hardwareMap).telemetry(telemetry).openCvPipeline(barcodeDetectorPipeline);

        pipelineAndCamera = pipelineBuilder.build();

        pipelineAndCamera.createAndRunPipeline();
    }

    @Override
    public void start() {
        super.start();

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

        if (operatorControls != null) {
            operatorControls.periodicTask();
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
                    case DETECT_BARCODE_DUCK_STORAGE:
                        setupDeliverDuckParkStorageWithBarcode();
                        break;
                    case DELIVER_DUCK_PARK_STORAGE:
                        setupDeliverDuckParkStorage();
                        break;
                    case PARK_WAREHOUSE:
                        setupParkWarehouse();
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

    protected void setupDeliverDuckParkStorage() {
        SequenceOfStates sequence = new SequenceOfStates(ticker, telemetry);

        // deliver duck program

        // Starting position: robot edge lined up on outer edge of tile closest to carousel

        // drive forward 14.5"

        State forwardFromWall = new TrajectoryFollowerState("ForwardFromWall",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.forward(14.5);

                return trajectoryBuilder.build();
            }
        };

        sequence.addSequential(forwardFromWall);

        // strafe towards wall (alliance dependent!) 23.25

        State strafeToWall = new TrajectoryFollowerState("StrafeToWall",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeLeft(23.25);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeRight(23.25);
                }

                return trajectoryBuilder.build();
            }
        };

        sequence.addSequential(strafeToWall);

        // backward 9.5" to engage with carousel

        State backToCarousel = new TrajectoryFollowerState("BackToCarousel",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.back(9.5);

                return trajectoryBuilder.build();
            }
        };

        sequence.addSequential(backToCarousel);

        // deliver duck

        State runCarousel = new StopwatchTimeoutSafetyState("RunCarousel",
                telemetry, ticker, TimeUnit.SECONDS.toMillis(4 * 1000)) {
            @Override
            public State doStuffAndGetNextState() {
                if (currentAlliance == Constants.Alliance.RED) {
                    carouselMechanism.spinRedForAuto();
                } else {
                    carouselMechanism.spinBlueForAuto();
                }

                return nextState;
            }
        };

        sequence.addSequential(runCarousel);

        // Wait for duck to be delivered
        sequence.addSequential(newMsDelayState("Wait for duck", 3500));

        // drive forward 17"

        State forwardToStorage = new TrajectoryFollowerState("ForwardToStorage",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.forward(18);

                return trajectoryBuilder.build();
            }
        };

        sequence.addSequential(forwardToStorage);

        // Stop running the carousel mechanism
        State stopCarousel = new State("Stop carousel", telemetry) {
            @Override
            public State doStuffAndGetNextState() {
                carouselMechanism.stop();

                return nextState;
            }

            @Override
            public void resetToStart() {

            }
        };

        sequence.addSequential(stopCarousel);

        sequence.addSequential(newDoneState("Done!"));

        stateMachine.addSequence(sequence);
    }

    protected void setupDeliverDuckParkStorageWithBarcode() {

        // deliver duck program

        // Starting position: robot edge lined up on outer edge of tile closest to carousel

        // Attempt to detect the duck

        BarcodeDetectorState detectionState = new BarcodeDetectorState();

        State backwardFromHub = setupAfterBarcodeDetectionStates(detectionState);

        // strafe towards wall (alliance dependent!) 23.25

        State strafeToWall = new TrajectoryFollowerState("StrafeToWall",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeLeft(23.25 + 24 + 3.25 + 2 + 3);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeRight(29.25 + 24 + 3.25 + 2 + 3);
                }

                return trajectoryBuilder.build();
            }
        };

        State squareUpA = new TrajectoryFollowerState("SquareUp1",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeLeft(7);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeRight(7);
                }

                return trajectoryBuilder.build();
            }
        };

        State squareUpB = new TrajectoryFollowerState("SquareUp2",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeRight(4);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeLeft(4);
                }

                return trajectoryBuilder.build();
            }
        };


        backwardFromHub.setNextState(strafeToWall);

        State squareUpPause = newMsDelayState("square up pause", 250);

        strafeToWall.setNextState(squareUpPause);

        squareUpPause.setNextState(squareUpA);
        squareUpA.setNextState(squareUpB);

        final State waitForDuck = setupCarouselStates(squareUpB);

        // drive forward 17"

        State forwardToStorage = new TrajectoryFollowerState("ForwardToStorage",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.forward(18 + 2.5 -1.75);

                return trajectoryBuilder.build();
            }
        };

        waitForDuck.setNextState(forwardToStorage);

        // Square up again
        State squareUp2A = new TrajectoryFollowerState("SquareUp2",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeLeft(7);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeRight(7);
                }

                return trajectoryBuilder.build();
            }
        };

        State squareUp2B = new TrajectoryFollowerState("SquareUp2",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeRight(4);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeLeft(4);
                }

                return trajectoryBuilder.build();
            }
        };

        State squareUpPause2 = newMsDelayState("square up pause", 250);

        squareUp2A.setNextState(squareUp2B);
        squareUp2B.setNextState(squareUpPause2);

        forwardToStorage.setNextState(squareUp2A);

        // Have we gone far enough?

        State makeSureInStorage = new TrajectoryFollowerState("MakeSureInStorage",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                final double distanceToWall = rearDistanceSensor.getDistance(DistanceUnit.INCH);

                // ideal distance from wall to put robot in center of storage is 28 inches

                final double distanceRemaining = PERFECT_WALL_STORAGE_DISTANCE - distanceToWall;

                int distanceToDrive = (int)distanceRemaining;

                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                Log.d(LOG_TAG, "Calculated distance  " + distanceToDrive + " to be in storage");

                if (Math.abs(distanceToDrive) > TOO_FAR_TO_DRIVE_INTO_STORAGE || distanceToDrive == 0) {
                    // do nothing
                    Log.d(LOG_TAG, "No need to drive further");

                    return null;
                } else if (distanceToDrive < 0) {
                    trajectoryBuilder.back(Math.abs(distanceToDrive));
                    Log.d(LOG_TAG, "Backwards " + Math.abs(distanceToDrive) + " to be in storage");
                } else if (distanceToDrive > 0) {
                    Log.d(LOG_TAG, "Forwards " + Math.abs(distanceToDrive) + " to be in storage");

                    trajectoryBuilder.forward(distanceToDrive);
                }

                return trajectoryBuilder.build();
            }
        };

        squareUpPause2.setNextState(makeSureInStorage);

        // Stop running the carousel mechanism
        State stopCarousel = new State("Stop carousel", telemetry) {
            @Override
            public State doStuffAndGetNextState() {
               carouselMechanism.stop();

                return nextState;
            }

            @Override
            public void resetToStart() {

            }
        };

        makeSureInStorage.setNextState(stopCarousel);
        
        State dropArmState = new RunnableState("drop arm", telemetry, 
                () -> freightManipulator.moveArmToStartPosition());

        stopCarousel.setNextState(dropArmState);

        dropArmState.setNextState(newDoneState("Done!"));

        stateMachine.setFirstState(detectionState);
    }

    private State setupCarouselStates(State strafeToWall) {
        // backward 9.5" to engage with carousel

        State backToCarouselPart1 = new TrajectoryFollowerState("BackToCarousel1",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.back(7);

                return trajectoryBuilder.build();
            }
        };

        State backToCarouselPart2 = new TrajectoryFollowerState("BackToCarousel",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.back(1);

                return trajectoryBuilder.build();
            }
        };

        State pauseABit = newMsDelayState("Pause a bit", 250);
        backToCarouselPart1.setNextState(pauseABit);
        pauseABit.setNextState(backToCarouselPart2);

        strafeToWall.setNextState(backToCarouselPart1);

        // deliver duck

        State runCarousel = new StopwatchTimeoutSafetyState("RunCarousel",
                telemetry, ticker, TimeUnit.SECONDS.toMillis(4 * 1000)) {
            @Override
            public State doStuffAndGetNextState() {
                if (currentAlliance == Constants.Alliance.RED) {
                    carouselMechanism.spinRedForAuto();
                } else {
                    carouselMechanism.spinBlueForAuto();
                }

                return nextState;
            }
        };

        backToCarouselPart2.setNextState(runCarousel);

        // Wait for duck to be delivered
        final State waitForDuck = newMsDelayState("Wait for duck", 3500 + (2 * 1000));
        runCarousel.setNextState(waitForDuck);
        return waitForDuck;
    }

    private double getDistanceForHubLevel() {

        final double distanceToFromHub = 15.5 - 9 + 2;

        switch (detectedPosition) {
            case LEFT: {
                return distanceToFromHub - 1;
            }
            case CENTER: {
                return distanceToFromHub - 1;
            }
            case RIGHT: {
                return distanceToFromHub;
            }
            default: {
                return distanceToFromHub;
            }
        }
    }

    @NonNull
    private State setupAfterBarcodeDetectionStates(final BarcodeDetectorState detectionState) {
        State armClearOfTseState = new RunnableState("Clear TSE", telemetry,
                () -> freightManipulator.moveArmToMiddleGoal());

        detectionState.setNextState(armClearOfTseState);

        // drive forward 14.5"

        State forwardFromWall = new TrajectoryFollowerState("ForwardFromWall",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.forward(14.5);

                return trajectoryBuilder.build();
            }
        };

        armClearOfTseState.setNextState(forwardFromWall);

        // Navigate to the hub

        // (1) We need to make up 31" (or so, probably less)
        //     (a) We're already at 14.5"
        // (2) Strafe 27" (red -> right, blue -> left)

        State strafeToHub = new TrajectoryFollowerState("StrafeToHub",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeRight(27 + 3.5 + 3);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeLeft(21 + 3.5 + 3);
                }

                return trajectoryBuilder.build();
            }
        };


        forwardFromWall.setNextState(strafeToHub);

        // Raise arm to detected level
        MoveArmToDetectedLevelState moveArmToDetectedLevelState = new MoveArmToDetectedLevelState();

        strafeToHub.setNextState(moveArmToDetectedLevelState);

        // (3) Forward 15.5"

        // FIXME: If we're going to score in the lower level, this probably needs to be a bit shorter!

        State forwardToHub = new TrajectoryFollowerState("ForwardToHub",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.forward(getDistanceForHubLevel());

                return trajectoryBuilder.build();
            }
        };

        State waitForArmState = newMsDelayState("Wait arm move", 750);

        moveArmToDetectedLevelState.setNextState(waitForArmState);

        waitForArmState.setNextState(forwardToHub);

        // (4) Release the freight
        RunnableState dropFreight = new RunnableState("Drop Freight", telemetry,
                () -> freightManipulator.spinOuttake());

        // (5) Wait for the freight to drop (a second/fraction of a second)
        forwardToHub.setNextState(dropFreight);

        State delayState = newMsDelayState("Wait freight", 1500);

        dropFreight.setNextState(delayState);

        // (6) Backwards 15.5" Pull back, at least to where we would've been for the old route

        // FIXME: If we're going to score in the lower level, this probably needs to be a bit shorter!

        State backwardFromHub = new TrajectoryFollowerState("BackwardFromHub",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.back(getDistanceForHubLevel());

                return trajectoryBuilder.build();
            }
        };

        delayState.setNextState(backwardFromHub);

        // Stop the outtake only after we back away, prevent freight manipulator
        // from pulling hub with it as we back away
        RunnableState stopOuttake = new RunnableState("Stop outtake", telemetry,
                () -> freightManipulator.stopIntake());

        backwardFromHub.setNextState(stopOuttake);

        State armClearOfTseState2 = new RunnableState("Clear TSE", telemetry,
                () -> freightManipulator.moveArmToMiddleGoal());

        stopOuttake.setNextState(armClearOfTseState2);

        return armClearOfTseState2;
    }

    protected void setupParkWarehouse() {
        final SequenceOfStates sequence = new SequenceOfStates(ticker, telemetry);

        // Starting position Back of robot towards warehouse, lined up with seam from third and fourth tile from carousel.

        // Put the arm in a safe position
        State armSafeState = new State("Arm safe", telemetry) {

            @Override
            public State doStuffAndGetNextState() {
                freightManipulator.moveToSafePosition();

                return nextState;
            }

            @Override
            public void resetToStart() {

            }
        };

        sequence.addSequential(armSafeState);

        // Strafe 18.5in away from wall.
        //

        State strafeFromWall = new TrajectoryFollowerState("StrafeFromWall",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                if (currentAlliance == Constants.Alliance.RED) {
                    trajectoryBuilder.strafeRight(18.5);
                } else {
                    // it's blue
                    trajectoryBuilder.strafeLeft(18.5);
                }

                return trajectoryBuilder.build();
            }
        };

        sequence.addSequential(strafeFromWall);

        // Backwards to the warehouse
        State backwardToWarehouse = new TrajectoryFollowerState("BackwardToWarehouse",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected Trajectory createTrajectory() {
                TrajectoryBuilder trajectoryBuilder = driveBase.trajectoryBuilder();

                trajectoryBuilder.back(80 - 10);

                return trajectoryBuilder.build();
            }
        };

        sequence.addSequential(backwardToWarehouse);

        sequence.addSequential(newDoneState("Done!"));

        stateMachine.addSequence(sequence);
    }

    protected void setupParkWarehouseSimple() {
        final TrajectoryAwareSequenceOfStates sequence = TrajectoryAwareSequenceOfStates.builder()
                .driveBase(driveBase).telemetry(telemetry).ticker(ticker).build();

        // Starting position Back of robot towards warehouse, lined up with seam from third and fourth tile from carousel.

        sequence.addRunnableStep("Arm safe", () -> freightManipulator.moveToSafePosition());

        sequence.addTrajectory("Strafe from wall", (t) -> {
            if (currentAlliance == Constants.Alliance.RED) {
                return t.strafeRight(18.5);
            }

            // it's blue
            return t.strafeLeft(18.5);
        });


        // Backwards to the warehouse
        sequence.addTrajectory("Back into warehouse", (t) -> t.back(80 - 10));

        sequence.addSequential(newDoneState("Done!"));

        stateMachine.addSequence(sequence);
    }

    /**
     * Creates an instance of the "delay" state which waits the number of seconds before
     * advancing to the next state
     */
    protected State newDelayState() {
        return newDelayState("start delay", initialDelaySeconds);
    }

    // TODO: Move to new model of controllers after first league meet
    protected RangeInput driverLeftStickX;

    protected RangeInput driverLeftStickY;

    protected RangeInput driverRightStickX;

    protected RangeInput driverRightStickY;

    protected RangeInput driveForwardReverse;

    protected RangeInput driveStrafe;

    protected RangeInput driveRotate;

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

    protected State newMsDelayState(String name, final int numberOfMillis) {
        return new StopwatchDelayState(name, telemetry, ticker, numberOfMillis, TimeUnit.MILLISECONDS);
    }

    protected State newDelayState(String name, final int numberOfSeconds) {
        return new State(name, telemetry) {

            private long startTime = 0;
            private long thresholdTimeMs = TimeUnit.SECONDS.toMillis(numberOfSeconds);

            @Override
            public void resetToStart() {
                startTime = 0;
            }

            @Override
            public State doStuffAndGetNextState() {
                if (startTime == 0) {
                    startTime = System.currentTimeMillis();
                    return this;
                }

                long now = System.currentTimeMillis();
                long elapsedMs = now - startTime;

                if (elapsedMs > thresholdTimeMs) {
                    return nextState;
                }

                telemetry.addData("04", "Delay: %d of %d ms", elapsedMs, thresholdTimeMs);
                return this;
            }
        };
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
                if (!issuedStop) {
                    driveBase.setMotorPowers(0, 0, 0, 0);

                    freightManipulator.stopIntake();

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

    class BarcodeDetectorState extends StopwatchTimeoutSafetyState {

        protected BarcodeDetectorState() {
            super("Detect bc", Auto.this.telemetry, ticker, TimeUnit.SECONDS.toMillis(4));
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut()) {
                resetTimer();
                Log.d(LOG_TAG, "Timed out looking for barcode, using level one");

                shutdownPipeline();

                return nextState;
            }

            if (!barcodeDetectorPipeline.isStartLookingForBarcode()) {
                barcodeDetectorPipeline.setStartLookingForBarcode(true);

                return this;
            }

            BarcodeDetectorPipeline.BarcodePosition detectedPosition = barcodeDetectorPipeline.getDetectedPosition();

            switch (detectedPosition) {
                case LEFT:
                    Log.d(LOG_TAG, "Detected level one");
                    driveTeamSignal.setDuckDetected(true);
                    shutdownPipeline();

                    Auto.this.detectedPosition = detectedPosition;

                    return nextState;
                case CENTER:
                    Log.d(LOG_TAG, "Detected level two");
                    driveTeamSignal.setDuckDetected(true);
                    shutdownPipeline();

                    Auto.this.detectedPosition = detectedPosition;

                    return nextState;
                case RIGHT:
                    Log.d(LOG_TAG, "Detected level three");
                    driveTeamSignal.setDuckDetected(true);
                    shutdownPipeline();

                    Auto.this.detectedPosition = detectedPosition;

                    return nextState;
                case UNKNOWN:
                default:
                    return this;
            }
        }

        private void shutdownPipeline() {
            barcodeDetectorPipeline.setStartLookingForBarcode(false);

            try {
                pipelineAndCamera.pauseViewport();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Error while pausing viewport, continuing...", t);
            }

            try {
                pipelineAndCamera.stopStreaming();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Error while stopping streaming, continuing...", t);
            }

            try {
                pipelineAndCamera.closeCameraDeviceAsync();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Error while closing camera, continuing...", t);
            }
        }
    }

    class MoveArmToDetectedLevelState extends State {
        protected MoveArmToDetectedLevelState() {
            super("Move arm to detected pos", Auto.this.telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            switch (detectedPosition) {
                case LEFT:
                    freightManipulator.moveArmToLowGoal();

                    return nextState;
                case CENTER:
                    freightManipulator.moveArmToMiddleGoal();

                    return nextState;
                case RIGHT:

                    freightManipulator.moveArmToTopGoal();

                    return nextState;
                case UNKNOWN:
                default:
                    freightManipulator.moveArmToTopGoal();

                    return nextState;
            }
        }

        @Override
        public void resetToStart() {

        }
    }

    class ArmToLevelOneState extends State {

        protected ArmToLevelOneState() {
            super("Arm l1", Auto.this.telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            freightManipulator.moveArmToLowGoal();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }
    }

    class ArmToLevelTwoState extends State {

        protected ArmToLevelTwoState() {
            super("Arm l2", Auto.this.telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            freightManipulator.moveArmToMiddleGoal();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }
    }

    class ArmToLevelThreeState extends State {

        protected ArmToLevelThreeState() {
            super("Arm l3", Auto.this.telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            freightManipulator.moveArmToTopGoal();

            return nextState;
        }

        @Override
        public void resetToStart() {

        }
    }
}