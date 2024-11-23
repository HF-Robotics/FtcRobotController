/*
 Copyright (c) 2024 The Tech Ninja Team (https://ftc9929.com)

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

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.roadrunner1.quickstart.MecanumDrive;
import com.hfrobots.tnt.corelib.roadrunner.ActionWithTimeout;
import com.hfrobots.tnt.util.Shared;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

@Autonomous(name = "ITDEEP2", group = "Autonomous")
public class IntoTheDeepAuto2 extends LinearOpMode {

    private Ticker ticker;

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

    private IntoTheDeepOperatorControls operatorControls;

    private IntoTheDeepDriverControls driverControls;

    private IntoTheDeepDriveTeamSignal driveTeamSignal;

    class SpecimenLiftUp extends ActionWithTimeout {
        private boolean initialized = false;

        SpecimenLiftUp() {
            super(TimeUnit.SECONDS.toMillis(5));
        }

        @Override
        protected void initialize() {
            specimenMechanism.goAboveHighChamber();
        }

        @Override
        protected boolean runUnderTimeout() {
            specimenMechanism.periodicTask();

            if (!specimenMechanism.isAboveHighChamber()) {
                return true;
            } else {
                return false;
            }
        }

        @Override
        protected void onTimeout() {
            // do nothing special
        }
    }

    class SpecimenLiftStowed extends ActionWithTimeout {
        private boolean initialized = false;

        SpecimenLiftStowed() {
            super(TimeUnit.SECONDS.toMillis(5));
        }

        @Override
        protected void initialize() {
            specimenMechanism.stowLift();
        }

        @Override
        protected boolean runUnderTimeout() {
            specimenMechanism.periodicTask();

            if (!specimenMechanism.isAtLowerLimit()) {
                return true;
            } else {
                return false;
            }
        }

        @Override
        protected void onTimeout() {
            // do nothing special

            // FIXME: Idle the lift?
        }
    }


    class AttachSpecimen extends ActionWithTimeout {

        AttachSpecimen() {
            super(TimeUnit.SECONDS.toMillis(2));
        }

        @Override
        protected void initialize() {
            specimenMechanism.attachSpecimen();
        }

        @Override
        protected boolean runUnderTimeout() {
            specimenMechanism.periodicTask();

            if (!specimenMechanism.currentStateIsIdle()) {
                // true causes the action to rerun
                return true;
            } else {
                return false;
            }
        }

        @Override
        protected void onTimeout() {
            // do nothing specific
        }
    }

    class UngripSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            specimenMechanism.openGripper();

            return false;
        }
    }

    @Override
    public void runOpMode() {
        initialize();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder moveTowardsSubmersible = drive.actionBuilder(initialPose)
                .lineToX(-24)
                .waitSeconds(1);

        TrajectoryActionBuilder slowlyApproachSubmersible = moveTowardsSubmersible.endTrajectory().fresh()
                .lineToX(-31);

        TrajectoryActionBuilder backAwayFromSubmerisble = slowlyApproachSubmersible.endTrajectory().fresh()
                .lineToX(-24);

        TrajectoryActionBuilder parkInObservationZone = backAwayFromSubmerisble.endTrajectory().fresh()
                .strafeTo(new Vector2d(-24, 48 + 9 + 24 + 8))
                .setTangent(Math.toRadians(0))
                .lineToX(- 5);

        while (!isStopRequested() && !opModeIsActive()) {
            loopInInit();
        }

        waitForStart();

        scoringMech.arm.stopShoulder();

        if (isStopRequested()) return;

        Action trajectoryActionChosen = moveTowardsSubmersible.build();

        // We can do this together....
        final Action moveToSubmersible = new ParallelAction(
                trajectoryActionChosen,
                new SpecimenLiftUp()
        );

        final Action moveToParkInObservationZone = new SequentialAction(
                backAwayFromSubmerisble.build(),
                new ParallelAction(
                        new SpecimenLiftStowed(),
                        parkInObservationZone.build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        moveToSubmersible,
                        slowlyApproachSubmersible.build(),
                        new AttachSpecimen(),
                        new UngripSpecimen(),
                        new SleepAction(2),
                        moveToParkInObservationZone
                )
        );
    }

    private void setupOperatorControls() {
        operatorControls = IntoTheDeepOperatorControls.builder().operatorGamepad(new NinjaGamePad(gamepad2))
                .scoringMech(scoringMech)
                .specimenMechanism(specimenMechanism)
                .noAutoSpecimenUpDown(true)
                .build();
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

    private void initialize() {
        Shared.withBetterErrorHandling(() -> {
            ticker = createAndroidTicker();

            scoringMech = new IntoTheDeepScoringMech(hardwareMap);

            scoringMech.arm.shoulderDown(0.15F);

            unstallArmTimer.start();

            specimenMechanism = SpecimenMechanism.builderFromHardwareMap(hardwareMap, telemetry).build();

            setupDriverControls();
            setupOperatorControls();

            driveTeamSignal = new IntoTheDeepDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);

        });
    }

    private boolean configLocked = false;

    private Stopwatch unstallArmTimer = Stopwatch.createUnstarted();

    private void loopInInit() {
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
}
