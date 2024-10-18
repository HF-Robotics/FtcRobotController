package com.hfrobots.tnt.season2425;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntoTheDeepScoringMech {
    Arm arm;

    public IntoTheDeepScoringMech(final HardwareMap hardwareMap) {
        this.arm = new Arm(hardwareMap);
    }

    public static class Arm {
        DcMotorEx shoulderMotor;

        Servo elbowServo;

        final double ELBOW_STOWED_SERVO_POSITION = 0.0D;

        final double ELBOW_UNSTOWED_SERVO_POSITION = .98D;

        Arm(final HardwareMap hardwareMap) {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulderMotor");
            elbowServo = hardwareMap.get(Servo.class, "elbowServo");
            elbowServo.setPosition(ELBOW_STOWED_SERVO_POSITION);
        }

        public void shoulderUp(float power) {
            shoulderMotor.setPower(power);
        }

        public void shoulderDown(float power) {
            shoulderMotor.setPower(-power);
        }

        public void stopShoulder() {
            shoulderMotor.setPower(0);
        }

        public void forearmOut() {
            elbowServo.setPosition(ELBOW_UNSTOWED_SERVO_POSITION);
        }

        public void forearmIn() {
            elbowServo.setPosition(ELBOW_STOWED_SERVO_POSITION);
        }
    }
}
