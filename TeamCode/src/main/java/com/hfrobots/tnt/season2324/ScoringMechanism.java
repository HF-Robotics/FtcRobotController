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

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.hfrobots.tnt.corelib.controllers.LinearLiftController;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.season2223.LiftMechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.Builder;
import lombok.Setter;

public class ScoringMechanism extends LinearLiftController {
    public static final double LIFT_FIRST_LINE_ENCODER_POS = 1632;
    public static final double K_P_UP_TO_LINE = .008;
    protected static Tunables goBilda26_1 = new LinearLiftController.Tunables(){
        @Override
        protected double getOpenLoopDownPowerRatio() {
            return 1.0;
        }

        @Override
        protected double getAntigravityFeedForward() {
            return 0.0;
        }

        protected double getKPLowerLimit() {
            return .005;
        }

        protected double getPidOutputLowerLimitMin() {
            return -1;
        }

        @Override
        protected int getUpperLimitEncoderPos() {
            return 2632;
        }

        @Override
        protected int getLowerLimitEncoderPos() {
            return 0;
        }

        @Override
        protected int getClosedLoopStallTimeoutMillis() {
            return 8000;
        }
    };

    @Override
    public void periodicTask() {
        super.periodicTask();

        if (isUnsafePressed()) {
            bucketTipServo.setPosition(BUCKET_TIP_SERVO_BOTTOM_POS);
        }

        if (isAtLowerLimit()) {
            if (!started) {
                bucketTipServo.setPosition(BUCKET_TIP_SERVO_INIT_POS);
            } else {
                bucketTipServo.setPosition(BUCKET_TIP_SERVO_BOTTOM_POS);
            }
        }

//        if (bucketTipButton != null && bucketTipButton.isPressed()) {
//            bucketTipServo.setPosition(BUCKET_TIP_DUMP_POS);
//        } else {
//            if (traveling) {
//                bucketTipServo.setPosition(BUCKET_TIP_SERVO_TRAVEL_POSITION);
//            } else {
//                if (!started) {
//                    bucketTipServo.setPosition(BUCKET_TIP_SERVO_INIT_POS);
//                } else {
//                    bucketTipServo.setPosition(BUCKET_TIP_SERVO_BOTTOM_POS);
//                }
//            }
//        }
    }

    public static ScoringMechanism.ScoringMechanismBuilder builderFromHardwareMap(
            final HardwareMap hardwareMap,
            final Telemetry telemetry) {
        DigitalChannel lowerLimit = hardwareMap.get(DigitalChannel.class, "liftLowLimitSwitch");

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo bucketTipServo = hardwareMap.get(Servo.class, "bucketTipServo");

        return ScoringMechanism.scoringMechanismBuilder().tunables(goBilda26_1)
                .liftMotor(NinjaMotor.asNeverest20(liftMotor))
                .bucketTipServo(bucketTipServo)
                .lowerLiftLimit(lowerLimit)
                .telemetry(telemetry);
    }

    @Override
    protected State preHandleButtons() {
        if (liftThrottleIsUp() || liftThrottleIsDown()) {
            this.bucketTipServo.setPosition(BUCKET_TIP_SERVO_TRAVEL_POSITION);
        }

        if (toFirstLineButton != null && toFirstLineButton.getRise()) {
            return goFirstLineState;
        }

        return null;
    }

    @Override
    protected void setupStateMachine(final Telemetry telemetry) {
        super.setupStateMachine(telemetry);

        // We want idle to be able to dump the bucket on command
        this.idleState = new CenterStageLiftIdleState(telemetry);

        goFirstLineState = new LiftGoFirstLineState(telemetry);
    }

    class CenterStageLiftIdleState extends LiftIdleState {
        CenterStageLiftIdleState(final Telemetry telemetry) {
            super(telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            // FIXME: Probably needs some safeties!
            if (bucketTipButton != null && bucketTipButton.isPressed()) {
                bucketTipServo.setPosition(BUCKET_TIP_DUMP_POS);
            } else {

                if (!started) {
                    bucketTipServo.setPosition(BUCKET_TIP_SERVO_INIT_POS);
                } else {
                    if (isUnsafePressed()) {
                        bucketTipServo.setPosition(BUCKET_TIP_SERVO_BOTTOM_POS);
                    } else {
                        bucketTipServo.setPosition(BUCKET_TIP_SERVO_TRAVEL_POSITION);
                    }
                }
            }

            return super.doStuffAndGetNextState();
        }
    }

    private final Servo bucketTipServo;

    @Setter
    private OnOffButton bucketTipButton;

    @Setter
    private DebouncedButton toFirstLineButton;

    private final static double BUCKET_TIP_SERVO_BOTTOM_POS = 0.7716;

    private final static double BUCKET_TIP_SERVO_TRAVEL_POSITION = .721678; // 0.79666; // .497223;
    private final static double BUCKET_TIP_DUMP_POS = .5894; // 1.0;
    private final static double BUCKET_TIP_SERVO_INIT_POS = .20389;

    @Setter
    private boolean started = false;

    private State goFirstLineState;

    @Builder(builderMethodName = "scoringMechanismBuilder")
    protected ScoringMechanism(Tunables tunables,
                                      RangeInput liftThrottle,
                                      DebouncedButton liftUpperLimitButton,
                                      DebouncedButton liftLowerLimitButton,
                                      DebouncedButton liftEmergencyStopButton,
                                      ExtendedDcMotor liftMotor,
                                      Servo bucketTipServo,
                                      DigitalChannel upperLiftLimit,
                                      DigitalChannel lowerLiftLimit,
                                      OnOffButton limitOverrideButton,
                                      Telemetry telemetry) {
        super(tunables, liftThrottle, liftUpperLimitButton, liftLowerLimitButton, liftEmergencyStopButton, liftMotor, upperLiftLimit, lowerLiftLimit, telemetry, limitOverrideButton);

        this.bucketTipServo = bucketTipServo;
        this.bucketTipServo.setPosition(BUCKET_TIP_SERVO_INIT_POS);
    }

    class LiftGoFirstLineState extends LiftClosedLoopState {

        LiftGoFirstLineState(Telemetry telemetry) {
            super("Lift-auto-small", telemetry, TimeUnit.SECONDS.toMillis(15)); // FIXME
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isTimedOut() && limitOverrideButton != null && !limitOverrideButton.isPressed()) {
                stopLift();
                Log.e(LOG_TAG, "Timed out while going to small junction");

                return transitionToState(idleState);
            }

            State fromButtonState = handleButtons();

            // Handle possible transitions back to open loop
            if (fromButtonState != null && fromButtonState != this) {
                Log.d(LOG_TAG, "Leaving closed loop for " + fromButtonState.getName());

                return transitionToState(fromButtonState);
            }

            if (!initialized) {
                Log.d(LOG_TAG, "Initializing PID for state " + getName());

                setupPidController(K_P_UP_TO_LINE);

                pidController.setOutputRange(-1, 1); // fix bouncing while descending
                pidController.setAbsoluteSetPoint(true); // MM

                pidController.setTarget(LIFT_FIRST_LINE_ENCODER_POS,
                        liftMotor.getCurrentPosition());

                initialized = true;
            }

            // closed loop control based on motor encoders

            double pidOutput = pidController.getOutput(liftMotor.getCurrentPosition());

            if (pidController.isOnTarget()) {
                Log.d(LOG_TAG, "Lift reached small target");

                stopLift();

                return transitionToState(idleState);
            }

            liftMotor.setPower(pidOutput);

            return this;
        }
    }
}
