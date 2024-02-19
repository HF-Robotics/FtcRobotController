package com.hfrobots.tnt.season2324;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.ftc9929.testing.fakes.FakeTelemetry;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.drive.FakeServo;
import com.ftc9929.testing.fakes.sensors.FakeDigitalChannel;
import com.hfrobots.tnt.season2223.Gripper;
import com.hfrobots.tnt.season2223.LiftMechanism;
import com.hfrobots.tnt.season2223.PowerPlayTestConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Before;
import org.junit.Test;

public class ScoringMechanismTest {
    private ScoringMechanism scoringMechanism;

    private FakeDcMotorEx liftMotor;

    private FakeOnOffButton unsafeButton = new FakeOnOffButton();

    private FakeRangeInput liftThrottle = new FakeRangeInput();

    private FakeOnOffButton autoRetractLiftButton = new FakeOnOffButton();

    private FakeOnOffButton liftToFirstLineButton = new FakeOnOffButton();;
    private FakeOnOffButton pixelReleaseButton = new FakeOnOffButton();;

    private FakeTelemetry telemetry = new FakeTelemetry();

    private FakeServo bucketTipServo;

    private FakeDigitalChannel lowerLimitSwitch;

    @Before
    public void setup() {
        final HardwareMap hardwareMap = CenterstageTestConstants.HARDWARE_MAP;
        scoringMechanism = ScoringMechanism.builderFromHardwareMap(hardwareMap, telemetry).build();
        scoringMechanism.setLiftThrottle(liftThrottle);
        scoringMechanism.setBucketTipButton(pixelReleaseButton);
        scoringMechanism.setLimitOverrideButton(unsafeButton);
        scoringMechanism.setLiftLowerLimitButton(autoRetractLiftButton.debounced());
        scoringMechanism.setToFirstLineButton(liftToFirstLineButton.debounced());

        scoringMechanism.setStarted(true);

        liftMotor = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "liftMotor");

        bucketTipServo = (FakeServo) hardwareMap.get(Servo.class, "bucketTipServo");

        lowerLimitSwitch = (FakeDigitalChannel) hardwareMap.get(DigitalChannel.class, "liftLowLimitSwitch");
    }

    @Test
    public void upAndBackCheckStowed() {
        // Since we don't yet auto-home at start, place the lift in that position "manually"
        lowerLimitSwitch.setState(false);
        scoringMechanism.periodicTask();
        assertEquals(bucketTipServo.getPosition(), ScoringMechanism.BUCKET_TIP_SERVO_BOTTOM_POS, 0.01);

        liftThrottle.setCurrentPosition(-1);
        scoringMechanism.periodicTask();
        lowerLimitSwitch.setState(true);
        scoringMechanism.periodicTask();
        assertTrue(scoringMechanism.currentStateIsOpenLoopUpOrDown());

        // If lift is moving, it should not let us dump a pixel
        checkBucketDumpBehavior(false);

        // Now the lift is not moving, let's check if we can dump the pixel...
        liftThrottle.setCurrentPosition(0);
        scoringMechanism.periodicTask();
        assertEquals(bucketTipServo.getPosition(), ScoringMechanism.BUCKET_TIP_SERVO_TRAVEL_POSITION, 0.01);
        checkBucketDumpBehavior(true);

        pressAutoRetractButton();
        assertTrue(scoringMechanism.currentStateIsClosedLoopDown());
        lowerLimitSwitch.setState(false);
        scoringMechanism.periodicTask();
        assertTrue(scoringMechanism.currentStateIsAtLowerLimit());
        assertEquals(bucketTipServo.getPosition(), ScoringMechanism.BUCKET_TIP_SERVO_BOTTOM_POS, 0.01);
    }

    private void pressAutoRetractButton() {
        autoRetractLiftButton.setPressed(false);
        scoringMechanism.periodicTask();
        autoRetractLiftButton.setPressed(true);
        scoringMechanism.periodicTask();
    }

    private void checkBucketDumpBehavior(boolean shouldDump) {
        pixelReleaseButton.setPressed(true);
        scoringMechanism.periodicTask();
        if (shouldDump) {
            assertEquals(bucketTipServo.getPosition(), ScoringMechanism.BUCKET_TIP_DUMP_POS, 0.01);
            pixelReleaseButton.setPressed(false);
            scoringMechanism.periodicTask();
            assertEquals(bucketTipServo.getPosition(), ScoringMechanism.BUCKET_TIP_SERVO_TRAVEL_POSITION, 0.01);
        } else {
            assertEquals(bucketTipServo.getPosition(), ScoringMechanism.BUCKET_TIP_SERVO_TRAVEL_POSITION, 0.01);
        }
        pixelReleaseButton.setPressed(false);
        scoringMechanism.periodicTask();
    }
}
