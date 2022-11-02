package com.hfrobots.tnt.season2223;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.drive.FakeServo;
import com.hfrobots.tnt.fakes.FakeNinjaGamePad;

import org.junit.Test;

public class GripperTest {
    private final FakeServo servo;

    private final Gripper gripper;

    private final FakeNinjaGamePad operatorGamepad;

    private final FakeOnOffButton openButton;

    private final FakeOnOffButton closeButton;

    public GripperTest() {
        servo = new FakeServo();

        gripper = new Gripper(servo);

        operatorGamepad = new FakeNinjaGamePad();

        openButton = (FakeOnOffButton)operatorGamepad.getAButton();

        closeButton = (FakeOnOffButton)operatorGamepad.getYButton();
    }

    @Test
    public void happyPath() {
        // When the gripper starts up, what position should it be in?

        assertTrue(gripper.isOpen());

        gripper.close(); // assert what, after this?
        assertTrue(gripper.isClosed());
        assertFalse(gripper.isOpen());

        gripper.open(); // assert what, after this?
        assertTrue(gripper.isOpen());
        assertFalse(gripper.isClosed());
    }

    @Test
    public void operatorControls() {
        OperatorControls operatorControls = OperatorControls.builder()
                .operatorGamepad(operatorGamepad)
                .gripper(gripper).build();

        pressButton(operatorControls, openButton);
        assertTrue(gripper.isOpen());

        pressButton(operatorControls, closeButton);
        assertTrue(gripper.isClosed());
    }

    private void pressButton(OperatorControls operatorControls, FakeOnOffButton button) {
        button.setPressed(false);
        operatorControls.periodicTask();
        button.setPressed(true);
        operatorControls.periodicTask();
    }
}
