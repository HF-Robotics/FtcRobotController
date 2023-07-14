package com.hfrobots.tnt.experiments;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.controllers.LinearLiftController;
import com.hfrobots.tnt.corelib.drive.ExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.hfrobots.tnt.season2223.Gripper;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Builder;

public class PowerPlayLiftController extends LinearLiftController {
    protected static Tunables goBilda26_1 = new Tunables(){

        @Override
        protected int getUpperLimitEncoderPos() {
            return 1800;
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

    private final Gripper gripper;

    public static PowerPlayLiftController.PowerPlayLiftControllerBuilder builderFromHardwareMap(final HardwareMap hardwareMap,
                                                                            final Telemetry telemetry) {
        DigitalChannel lowerLimit = hardwareMap.get(DigitalChannel.class, "lowLimitSwitch");
        DigitalChannel higherLimit = hardwareMap.get(DigitalChannel.class, "highLimitSwitch");

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        return PowerPlayLiftController.powerPlayLiftBuilder().tunables(goBilda26_1)
                .liftMotor(NinjaMotor.asNeverest20(liftMotor))
                .lowerLiftLimit(lowerLimit)
                .upperLiftLimit(higherLimit)
                .telemetry(telemetry);
    }

    @Override
    protected void setupStateMachine(Telemetry telemetry) {
        super.setupStateMachine(telemetry);

        // We want to close the gripper before going to the low limit
        goLowerLimitState = new LiftCloseGripperAndGoLowerLimitState(telemetry);
    }

    @Builder(builderMethodName = "powerPlayLiftBuilder")
    protected PowerPlayLiftController(Tunables tunables,
                                      RangeInput liftThrottle,
                                      DebouncedButton liftUpperLimitButton,
                                      DebouncedButton liftLowerLimitButton,
                                      DebouncedButton liftEmergencyStopButton,
                                      ExtendedDcMotor liftMotor,
                                      DigitalChannel upperLiftLimit,
                                      DigitalChannel lowerLiftLimit,
                                      OnOffButton limitOverrideButton,
                                      Telemetry telemetry,
                                      Gripper gripper) {
        super(tunables, liftThrottle, liftUpperLimitButton, liftLowerLimitButton, liftEmergencyStopButton, liftMotor, upperLiftLimit, lowerLiftLimit, telemetry, limitOverrideButton);

        this.gripper = gripper;
    }

    private class LiftCloseGripperAndGoLowerLimitState extends LiftGoLowerLimitState {
        LiftCloseGripperAndGoLowerLimitState(final Telemetry telemetry) {
            super(telemetry);
        }

        @Override
        protected void initialize() {
            super.initialize();

            gripper.close();
        }

        @Override
        protected void reachedLowerLimit() {
            super.reachedLowerLimit();

            gripper.open();
        }
    }
}
