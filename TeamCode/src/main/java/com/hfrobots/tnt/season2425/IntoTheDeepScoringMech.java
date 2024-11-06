package com.hfrobots.tnt.season2425;

import com.ftc9929.corelib.control.OnOffButton;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import lombok.Setter;

public class IntoTheDeepScoringMech {
    public static final int SHOULDER_MOTOR_UPPER_LIMIT = 850;
    Arm arm;

    @Setter
    private OnOffButton unsafeButton;

    public IntoTheDeepScoringMech(final HardwareMap hardwareMap) {
        this.arm = new Arm(hardwareMap);
    }

    public class Arm {
        public static final float SHOULDER_FEED_FORWARD = 0.24F;
        DcMotorEx shoulderMotor;

        Servo elbowServo;

        Servo intakeServo;

        final double ELBOW_STOWED_SERVO_POSITION = 0.0D;

        final double ELBOW_UNSTOWED_SERVO_POSITION = .98D;

        Arm(final HardwareMap hardwareMap) {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulderMotor");
            elbowServo = hardwareMap.get(Servo.class, "elbowServo");
            elbowServo.setPosition(ELBOW_STOWED_SERVO_POSITION);

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

        public void forearmOut() {
            double elbowPosition = elbowServo.getPosition();
            elbowPosition += .035;
            elbowPosition = Math.min(elbowPosition, ELBOW_UNSTOWED_SERVO_POSITION);

            elbowServo.setPosition(elbowPosition);
        }

        public void forearmIn() {
            double elbowPosition = elbowServo.getPosition();
            elbowPosition -= .035;
            elbowPosition = Math.max(elbowPosition, ELBOW_STOWED_SERVO_POSITION);

            elbowServo.setPosition(elbowPosition);
        }

        public void intakeSample() {
            intakeServo.setPosition(1);
        }

        public void outtakeSample() {
            intakeServo.setPosition(0);
        }

        public void stopIntake() {
            //intakeServo.setPower(0);
        }
    }

    boolean isUnsafePressed() {
        return unsafeButton != null && unsafeButton.isPressed();
    }
}
