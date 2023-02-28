package com.hfrobots.tnt.season2223;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.testing.fakes.FakeTelemetry;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.drive.FakeServo;
import com.hfrobots.tnt.season2122.CarouselMechanism;
import com.hfrobots.tnt.season2122.FreightFrenzyTestConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Before;
import org.junit.Test;

public class LiftTest {
    public static final int ENCODERS_PER_CYCLE_FREE_SPEED = 42;
    private LiftMechanism liftMechanism;

    private FakeDcMotorEx liftMotor;

    private FakeOnOffButton limitOverrideButton = new FakeOnOffButton();
    private FakeRangeInput liftThrottle = new FakeRangeInput();
    private FakeOnOffButton liftGoSmallButton = new FakeOnOffButton();
    private FakeOnOffButton liftGoMediumButton = new FakeOnOffButton();;
    private FakeOnOffButton liftLowerLimitButton = new FakeOnOffButton();;
    private FakeOnOffButton liftUpperLimitButton = new FakeOnOffButton();;
    private FakeOnOffButton liftEmergencyStopButton = new FakeOnOffButton();;
    private FakeOnOffButton coneGrabAutomation = new FakeOnOffButton();;

    private FakeServo gripperServo;
    private Gripper gripper;

    @Before
    public void setup() {
        final HardwareMap hardwareMap = PowerPlayTestConstants.HARDWARE_MAP;
        gripperServo = (FakeServo)hardwareMap.get(Servo.class, "gripperServo");
        gripper = new Gripper(gripperServo);

        LiftMechanism.LiftMechanismBuilder builder = LiftMechanism.builderFromHardwareMap(hardwareMap, new FakeTelemetry());
        builder.gripper(gripper);

        liftMechanism = builder.limitOverrideButton(limitOverrideButton).build();
        liftMechanism.setLiftThrottle(liftThrottle);
        liftMechanism.setLiftGoSmallButton(liftGoSmallButton.debounced());
        liftMechanism.setLiftGoMediumButton(liftGoMediumButton.debounced());
        liftMechanism.setLiftLowerLimitButton(liftLowerLimitButton.debounced());
        liftMechanism.setLiftUpperLimitButton(liftUpperLimitButton.debounced());
        liftMechanism.setLiftEmergencyStopButton(liftEmergencyStopButton.debounced());
        liftMechanism.setAutoGrabConeButton(coneGrabAutomation.debounced());
        liftMotor = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "liftMotor");
    }

    @Test
    public void autoGrabPid() throws InterruptedException {
        liftMechanism.grabAndLiftCone();

        liftMechanism.periodicTask();
        Thread.sleep(750);

        for (int i = 0; i < 200; i++) {
            liftMechanism.periodicTask();
            liftMotor.setCurrentPosition(liftMotor.getCurrentPosition() + ENCODERS_PER_CYCLE_FREE_SPEED);
            System.out.println(liftMechanism.getCurrentStateName());
        }
    }
}
