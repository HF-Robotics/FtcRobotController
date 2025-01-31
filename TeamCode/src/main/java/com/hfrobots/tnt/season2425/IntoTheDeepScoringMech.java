package com.hfrobots.tnt.season2425;

import com.ftc9929.corelib.control.OnOffButton;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Setter;

public class IntoTheDeepScoringMech {
    public static final int SHOULDER_MOTOR_UPPER_LIMIT = 850;
    public static final double FOREARM_SERVO_OUT_INCREMENT = .008/2;
    public static final double FOREARM_SERVO_IN_INCREMENT = .003;

    public static final double FOREARM_SERVO_PARALLEL_GROUND_POS = .560;

    Arm arm;

    @Setter
    private OnOffButton unsafeButton;

    private final Telemetry telemetry;

    public IntoTheDeepScoringMech(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.arm = new Arm(hardwareMap);
        this.telemetry = telemetry;
    }

    public class Arm {
        public static final float SHOULDER_FEED_FORWARD = 0.24F;
        private static final boolean DO_PWM_SHUTDOWN_STUFF = false;
        DcMotorEx shoulderMotor;

        Servo elbowServo;

        Servo intakeServo;

        final double ELBOW_STOWED_SERVO_POSITION = 0.0D;

        final double ELBOW_UNSTOWED_SERVO_POSITION = .82D;

        Arm(final HardwareMap hardwareMap) {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulderMotor");
            elbowServo = hardwareMap.get(Servo.class, "elbowServo");
            currentForearmServoPos =   ELBOW_STOWED_SERVO_POSITION;
            maintainForearmPosition();

            intakeServo = hardwareMap.get(Servo.class, "intakeServo");
            outtakeSample();
        }

        private Integer positionBeforeStopping = null;

        public void shoulderUp(float power) {
            setupShoulderMotorForMovement();

            if (!isUnsafePressed() && shoulderMotor.getCurrentPosition() >= SHOULDER_MOTOR_UPPER_LIMIT) {
                positionBeforeStopping = SHOULDER_MOTOR_UPPER_LIMIT;

                stopShoulder();

                return;
            }

            if (!isUnsafePressed()) {
                if (currentForearmServoPos > 0.05) {
                    power = power * .35F;
                }
            }

            shoulderMotor.setPower(power);
        }

        private void setupShoulderMotorForMovement() {
            positionBeforeStopping = null;

            if (isUnsafePressed()) {
                shoulderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void shoulderDown(float power) {
            setupShoulderMotorForMovement();


            if (!isUnsafePressed()) {
                if (currentForearmServoPos > 0.05) {
                    power = power * .15F;
                }

                power = power * .25F;
            }

            shoulderMotor.setPower(-power);
        }

        public void stopShoulder() {
            if (isUnsafePressed()) {
                shoulderUp(SHOULDER_FEED_FORWARD);
            } else {
                if (positionBeforeStopping == null) {
                    positionBeforeStopping = shoulderMotor.getCurrentPosition();
                }

                shoulderMotor.setTargetPosition(positionBeforeStopping);
                shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulderMotor.setPower(SHOULDER_FEED_FORWARD);
            }
        }

        public void setDeadStop() {
            shoulderMotor.setPower(0);
        }

        double dynamicUnstowedServoPosition = ELBOW_UNSTOWED_SERVO_POSITION;

        private void evaluateElbowStowedServoPosition() {
            dynamicUnstowedServoPosition = ELBOW_UNSTOWED_SERVO_POSITION;

            if (shoulderMotor.getCurrentPosition() < 60) {
                dynamicUnstowedServoPosition = FOREARM_SERVO_PARALLEL_GROUND_POS;

            }
        }

        public void forearmOut() {
            double elbowPosition = elbowServo.getPosition();
            elbowPosition += FOREARM_SERVO_OUT_INCREMENT;
            elbowPosition = Math.min(elbowPosition, dynamicUnstowedServoPosition);

            currentForearmServoPos = elbowPosition;

            if (!isUnsafePressed() && DO_PWM_SHUTDOWN_STUFF &&
                Math.abs(elbowPosition - dynamicUnstowedServoPosition) < .02) {
                elbowServo.getController().pwmDisable();
            } else {
                if (DO_PWM_SHUTDOWN_STUFF) {
                    elbowServo.getController().pwmEnable();
                }

                elbowServo.setPosition(elbowPosition);
            }

            telemetry.addData("elbow-pos: ", elbowPosition);


        }

        // FIXME: Hack, hack (or maybe a mitigation?)

        public void maintainForearmPosition() {
            evaluateElbowStowedServoPosition();

            if (dynamicUnstowedServoPosition < elbowServo.getPosition()) {
                currentForearmServoPos = dynamicUnstowedServoPosition;
            }

            if (DO_PWM_SHUTDOWN_STUFF && elbowServo.getController().getPwmStatus() == ServoController.PwmStatus.ENABLED) {
                elbowServo.setPosition(currentForearmServoPos);
            } else {
                elbowServo.setPosition(currentForearmServoPos);
            }
        }

        private double currentForearmServoPos;

        public void forearmIn() {
            if (DO_PWM_SHUTDOWN_STUFF) {
                elbowServo.getController().pwmEnable();
            }

            double elbowPosition = elbowServo.getPosition();
            elbowPosition -= FOREARM_SERVO_IN_INCREMENT;
            elbowPosition = Math.max(elbowPosition, ELBOW_STOWED_SERVO_POSITION);
            currentForearmServoPos = elbowPosition;

            elbowServo.setPosition(elbowPosition);
            telemetry.addData("elbow-pos: ", elbowPosition);
        }

        public void intakeSample() {
            intakeServo.setPosition(1);
        }

        public void outtakeSample() {
            intakeServo.setPosition(0);
        }

        public void maintainIntakeServoPos() {
            intakeServo.setPosition(intakeServo.getPosition());
        }

        public void stopIntake() {
            //intakeServo.setPower(0);
        }
    }

    boolean isUnsafePressed() {
        return unsafeButton != null && unsafeButton.isPressed();
    }
}
