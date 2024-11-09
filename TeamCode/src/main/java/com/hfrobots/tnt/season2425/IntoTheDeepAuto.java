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

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.google.common.base.Optional;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.google.common.collect.Sets;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.mecanum.MultipleTrajectoriesFollowerState;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveBase;
import com.hfrobots.tnt.corelib.state.ReadyCheckable;
import com.hfrobots.tnt.season2324.Shared;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Set;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "00 ITDEEP Auto", preselectTeleOp = ITDEEP_TELE_OP)
public class IntoTheDeepAuto extends OpMode {
    private Ticker ticker;

    private RoadRunnerMecanumDriveBase driveBase;

    private StateMachine stateMachine;

    // Used to ensure that states with circular dependencies are setup correctly
    private final Set<ReadyCheckable> readyCheckables = Sets.newHashSet();
    private IntoTheDeepScoringMech scoringMech;
    private SpecimenMechanism specimenMechanism;

    // FIXME: The tasks our robot knows how to do - rename these to
    //  something meaningful for the season!
    private enum Task {
        HANG_SPECIMEN("Hang specimen"),
        HANG_SPECIMEN_NO_PARK("Hang specimen - no park"),
        HANG_SPECIMEN_NET_SIDE("Hang specimen - net side");

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

    private IntoTheDeepDriveConstants driveConstants;

    private IntoTheDeepOperatorControls operatorControls;

    private IntoTheDeepDriverControls driverControls;

    private VisionPortal visionPortal;

    private IntoTheDeepDriveTeamSignal driveTeamSignal;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            scoringMech = new IntoTheDeepScoringMech(hardwareMap);

            scoringMech.arm.shoulderDown(0.15F);

            unstallArmTimer.start();

            specimenMechanism = SpecimenMechanism.builderFromHardwareMap(hardwareMap, telemetry).build();

            setupDriverControls();
            setupOperatorControls();

            //setupVisionPortal(hardwareMap);

            driveTeamSignal = new IntoTheDeepDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

            driveConstants = new IntoTheDeepDriveConstants();

            // MM: FIXME - how are these sitting?
            IMU.Parameters imuParameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

            driveBase = new RoadRunnerMecanumDriveBase(hardwareMap,
                    driveConstants, Optional.of(imuParameters));

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
        scoringMech.arm.stopShoulder();

        Shared.withBetterErrorHandling(() -> {
            super.start();
            if (visionPortal != null) {
                visionPortal.stopLiveView();
            }
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

    private Stopwatch unstallArmTimer = Stopwatch.createUnstarted();

    @Override
    public void init_loop() {
        if (unstallArmTimer.isRunning() && unstallArmTimer.elapsed(TimeUnit.SECONDS) > 30) {
            scoringMech.arm.setDeadStop();
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
                    case HANG_SPECIMEN:
                        setupTaskHangSpecimen();
                        break;
                    case HANG_SPECIMEN_NO_PARK:
                        setupTaskHangSpecimenNoPark();
                        break;
                    case HANG_SPECIMEN_NET_SIDE:
                        setupTaskHangSpecimenGoToNets();
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

            specimenMechanism.periodicTask();
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

    protected void setupTaskHangSpecimen() {
        // Alternatively, for something straightforward you can do sequentials, like this:
        SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        setupCommonSpecimenHang(sequenceOfStates, false);

        final State toObservationZone = new MultipleTrajectoriesFollowerState("To observation zone",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                driveBase.resetLocalizer();

                addTrajectoryProvider("to bar", (t) -> t.strafeLeft(48 + 9));
            }
        };

        final State pullToPark = new MultipleTrajectoriesFollowerState("park",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                driveBase.resetLocalizer();

                addTrajectoryProvider("to bar", (t) -> t.forward(17));
            }
        };

        sequenceOfStates.addSequential(toObservationZone);
        sequenceOfStates.addSequential(pullToPark);
        sequenceOfStates.addSequential(newDoneState("Done!"));
        stateMachine.addSequence(sequenceOfStates);
    }

    protected void setupTaskHangSpecimenNoPark() {
        // Alternatively, for something straightforward you can do sequentials, like this:
        SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        setupCommonSpecimenHang(sequenceOfStates, false);

        sequenceOfStates.addSequential(newDoneState("Done!"));
        stateMachine.addSequence(sequenceOfStates);
    }

    protected void setupTaskHangSpecimenGoToNets() {
        // Alternatively, for something straightforward you can do sequentials, like this:
        SequenceOfStates sequenceOfStates = new SequenceOfStates(ticker, telemetry);

        setupCommonSpecimenHang(sequenceOfStates, true);

        final State moveToNets = new MultipleTrajectoriesFollowerState("move to nets",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                driveBase.resetLocalizer();

                addTrajectoryProvider("to bar", (t) -> t.strafeRight(48 +9));
            }
        };

        sequenceOfStates.addSequential(moveToNets);

        sequenceOfStates.addSequential(newDoneState("Done!"));
        stateMachine.addSequence(sequenceOfStates);
    }

    private void setupCommonSpecimenHang(final SequenceOfStates sequenceOfStates,
                                         boolean needsStrafeToAlignWithChamber) {

        final State moveBackwardsFromWall = new MultipleTrajectoriesFollowerState("Move backwards from wall",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                driveBase.resetLocalizer();

                addTrajectoryProvider("Off wall", (t) -> t.back(27));
            }
        };

        final State raiseGripper = new RunnableState("raiseGripper", telemetry, () -> {
            specimenMechanism.goAboveHighChamber();
        });

        final double distanceToBar = needsStrafeToAlignWithChamber ? 4 + 1 : 3 + 1;

        final State moveBackwardsToBar = new MultipleTrajectoriesFollowerState("Move backwards to bar",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                driveBase.resetLocalizer();

                addTrajectoryProvider("to bar", (t) -> t.back(distanceToBar));
            }
        };

        final State hookSpecimen = new RunnableState("hook specimen", telemetry, () -> {
            specimenMechanism.attachSpecimen();
        });

        final State openGripper = new RunnableState("open gripper", telemetry, () -> {
            specimenMechanism.openGripper();
        });

        final State moveForwardFromBar = new MultipleTrajectoriesFollowerState("Move forward from bar",
                telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
            @Override
            protected void createTrajectoryProviders() {
                driveBase.resetLocalizer();

                addTrajectoryProvider("from bar", (t) -> t.forward(3));
            }
        };

        final State lowerSpecimenMechanism = new RunnableState("lower specimen lift", telemetry, () -> {
            specimenMechanism.stowLift();
        });

        sequenceOfStates.addSequential(moveBackwardsFromWall);
        sequenceOfStates.addSequential(raiseGripper);
        sequenceOfStates.addWaitStep("Wait for gripper to raise", 2, TimeUnit.SECONDS);

        if (needsStrafeToAlignWithChamber) {
            final State alignWithChambers = new MultipleTrajectoriesFollowerState("align with chambers",
                    telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(20 * 1000)) {
                @Override
                protected void createTrajectoryProviders() {
                    driveBase.resetLocalizer();

                    addTrajectoryProvider("to bar", (t) -> t.strafeLeft(16));
                }
            };

            sequenceOfStates.addSequential(alignWithChambers);
        }

        sequenceOfStates.addSequential(moveBackwardsToBar);
        sequenceOfStates.addSequential(hookSpecimen);
        sequenceOfStates.addWaitStep("Wait for gripper to lower", 1, TimeUnit.SECONDS);
        sequenceOfStates.addSequential(openGripper);
        sequenceOfStates.addWaitStep("wait for gripper to open", 2, TimeUnit.SECONDS);
        sequenceOfStates.addSequential(moveForwardFromBar);
        sequenceOfStates.addSequential(lowerSpecimenMechanism);
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
        operatorControls = IntoTheDeepOperatorControls.builder().operatorGamepad(new NinjaGamePad(gamepad2))
                .scoringMech(scoringMech)
                .specimenMechanism(specimenMechanism)
                .noAutoSpecimenUpDown(true)
                .build();
    }
}