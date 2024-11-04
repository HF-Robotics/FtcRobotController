package com.hfrobots.tnt.season2425;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class IntoTheDeepScoringMech {
    Arm arm;

    public IntoTheDeepScoringMech(final HardwareMap hardwareMap) {
        this.arm = new Arm(hardwareMap);
    }

    public static class Arm {
        DcMotorEx shoulderMotor;

        Servo elbowServo;

        Servo intakeServo;

        final double ELBOW_STOWED_SERVO_POSITION = 0.0D;

        final double ELBOW_UNSTOWED_SERVO_POSITION = .98D;

        final double ARM_ANTI_GRAVITY_FEED_FORWARD = 0;

        Arm(final HardwareMap hardwareMap) {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulderMotor");
            elbowServo = hardwareMap.get(Servo.class, "elbowServo");
            elbowServo.setPosition(ELBOW_STOWED_SERVO_POSITION);

            intakeServo = hardwareMap.get(Servo.class, "intakeServo");
            outtakeSample();
        }

        public void shoulderUp(float power) {
            shoulderMotor.setPower(power);
        }

        public void shoulderDown(float power) {
            shoulderMotor.setPower(-power);
        }

        public void stopShoulder() {
            if (forearmIsExtended) {
                shoulderUp(0.24F);
            } else {
                shoulderMotor.setPower(0);
            }
        }

        private boolean forearmIsExtended = false;

        public void forearmOut() {
            forearmIsExtended = true;
            double elbowPosition = elbowServo.getPosition();
            elbowPosition += .035;
            elbowPosition = Math.min(elbowPosition, ELBOW_UNSTOWED_SERVO_POSITION);

            elbowServo.setPosition(elbowPosition);
        }

        public void forearmIn() {
            forearmIsExtended = false;
            elbowServo.setPosition(ELBOW_STOWED_SERVO_POSITION);
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
}
