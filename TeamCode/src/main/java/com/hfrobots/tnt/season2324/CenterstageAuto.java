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

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.google.common.base.Optional;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.mecanum.DriveConstants;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveBase;
import com.hfrobots.tnt.corelib.drive.mecanum.util.AxisDirection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="00 CS Auto")
public class CenterstageAuto extends OpMode {
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

    private Ticker ticker;

    private RoadRunnerMecanumDriveBase driveBase;

    private StateMachine stateMachine;

    private CenterstageOperatorControls operatorControls;

    private CenterstageDriverControls driverControls;

    private CenterstageDriveConstants driveConstants;

    // The tasks our robot knows how to do - rename these to something meaningful for the season!
    private enum TaskChoice {
        SIT_THERE_MOTIONLESS("Sit and do nothing");

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
        ticker = createAndroidTicker();

        setupDriverControls();
        setupOperatorControls();

        driveConstants = new CenterstageDriveConstants();
        driveBase = new RoadRunnerMecanumDriveBase(hardwareMap,
                driveConstants,
                Optional.of(AxisDirection.POS_X));

        stateMachine = new StateMachine(telemetry);
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
        NinjaGamePad operatorsGamepad = new NinjaGamePad(gamepad2);

        operatorControls = CenterstageOperatorControls.builder()
                .operatorGamepad(operatorsGamepad).build();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    private boolean configLocked = false;

    @Override
    public void init_loop() {
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

        updateTelemetry(telemetry);
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
        try {
            if (!stateMachineSetup) {
                // FIXME: MM - There's a refactoring here that we should show the team!

                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                TaskChoice selectedTaskChoice = possibleTaskChoices[selectedTaskIndex];

                switch (selectedTaskChoice) {
                    case SIT_THERE_MOTIONLESS:
                        setupSitMotionlessStateMachine();
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

    protected void setupSitMotionlessStateMachine() {

        final SequenceOfStates sequence = new SequenceOfStates(ticker, telemetry);

        // Add a bunch of steps here!

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
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0);

        private static PIDCoefficients TRANSLATIONAL_PID_COEFFICIENTS = new PIDCoefficients(4.2D, 0, 0);

        private static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0.295D, 0, 0);

        private static double ENCODER_PPR = 384.5;

        private static double WHEEL_RADIUS_IN = 1.89;

        private static double TRACK_WIDTH_IN = 8.5;

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
}