package com.hfrobots.tnt.season2425;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntoTheDeepScoringMech {
    Arm arm;

    public IntoTheDeepScoringMech(final HardwareMap hardwareMap) {
        this.arm = new Arm(hardwareMap);
    }

    public static class Arm {
        DcMotorEx shoulderMotor;

        Arm(final HardwareMap hardwareMap) {
            shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulderMotor");
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

    }
}
