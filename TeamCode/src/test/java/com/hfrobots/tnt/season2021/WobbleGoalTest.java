/*
 Copyright (c) 2021 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season2021;

import com.ftc9929.corelib.control.ToggledButton;
import com.ftc9929.testing.fakes.FakeTelemetry;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.drive.FakeServo;
import com.ftc9929.testing.fakes.sensors.FakeDigitalChannel;
import com.google.common.testing.FakeTicker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

public class WobbleGoalTest {
    private FakeDcMotorEx shoulderMotor;

    private FakeServo gripperServo;

    private WobbleGoal wobbleGoal;

    private FakeDigitalChannel placeLimitSwitch;

    private FakeDigitalChannel stowLimitSwitch;

    private FakeTicker fakeTicker = new FakeTicker();

    private FakeTelemetry fakeTelemetry = new FakeTelemetry();

    private FakeRangeInput shoulderThrottle = new FakeRangeInput();

    private FakeOnOffButton gripperButton = new FakeOnOffButton();
    private ToggledButton gripperToggle;

    private FakeOnOffButton unsafe = new FakeOnOffButton();

    @Before
    public void setup() {
        wobbleGoal = WobbleGoal.builder()
                .hardwareMap(UltimateGoalTestConstants.HARDWARE_MAP)
                .telemetry(fakeTelemetry).ticker(fakeTicker).build();

        gripperToggle = new ToggledButton(gripperButton);
        wobbleGoal.setGripperButton(gripperToggle);

        wobbleGoal.setUnsafe(unsafe);

        wobbleGoal.setShoulderThrottle(shoulderThrottle);

        shoulderMotor = (FakeDcMotorEx) UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "shoulderMotor");

        gripperServo = (FakeServo) UltimateGoalTestConstants.HARDWARE_MAP.get(Servo.class, "gripperServo");

        placeLimitSwitch = (FakeDigitalChannel) UltimateGoalTestConstants.HARDWARE_MAP.get(DigitalChannel.class, "placeLimitSwitch");

        stowLimitSwitch = (FakeDigitalChannel) UltimateGoalTestConstants.HARDWARE_MAP.get(DigitalChannel.class, "stowLimitSwitch");
    }

    @Test
    public void fullRangeTest() {
        // Start with assuming that the mechanism has been placed in the stowed state for match start

        assertEquals(WobbleGoal.MotionState.class, wobbleGoal.getCurrentState().getClass());

        stowLimitSwitch.setState(false);

        // Try to move "backwards", this should not move the arm, but you should end up in stow
        commandStowDirection();
        wobbleGoal.periodicTask();

        assertEquals(WobbleGoal.StowState.class, wobbleGoal.getCurrentState().getClass());

        // Still try to move "backwards", this should not move the arm, but you should stay in stow
        commandStowDirection();
        wobbleGoal.periodicTask();

        assertEquals(WobbleGoal.StowState.class, wobbleGoal.getCurrentState().getClass());

        // Attempt to toggle the servo, what should happen?

        double initialServoPos = gripperServo.getPosition();

        causeButtonToRiseForNextPeriodTask(gripperButton);
        wobbleGoal.periodicTask();

        assertNotEquals(initialServoPos, gripperServo.getPortNumber(), 0.05);

        // Move "forwards", what state should the wobble goal mechanism be in?

        commandPlaceDirection();
        wobbleGoal.periodicTask();

        assertEquals(WobbleGoal.MotionState.class, wobbleGoal.getCurrentState().getClass());

        // Attempt to toggle the servo, what should happen in this state?

        initialServoPos = gripperServo.getPosition();

        causeButtonToRiseForNextPeriodTask(gripperButton);
        wobbleGoal.periodicTask();

        assertEquals(initialServoPos, gripperServo.getPosition(), 0.05);

        // Continue to move "forwards" until the arm activates the "placed" limit switch
        // what should happen?

        commandPlaceDirection();
        placeLimitSwitch.setState(false);
        wobbleGoal.periodicTask();

        assertEquals(WobbleGoal.PlaceState.class, wobbleGoal.getCurrentState().getClass());

        // What can be done with the servo when the arm is in this position/state?

        causeButtonToRiseForNextPeriodTask(gripperButton);
        wobbleGoal.periodicTask();

        assertEquals(WobbleGoal.CLOSED_GRIPPER_POS, gripperServo.getPosition(), 0.05);

        // Toggle the gripper opposite
        initialServoPos = gripperServo.getPosition();
        causeButtonToRiseForNextPeriodTask(gripperButton);
        wobbleGoal.periodicTask();

        assertNotEquals(initialServoPos, gripperServo.getPosition(), 0.05); // FIXME

        // Move "backwards", what state does the mechanism end up in? (I think, all things that
        // can be done in that state have been tested by this point!

        commandStowDirection();
        wobbleGoal.periodicTask();

        assertEquals(WobbleGoal.MotionState.class, wobbleGoal.getCurrentState().getClass()); // FIXME
    }

    private void commandStowDirection() {
        shoulderThrottle.setCurrentPosition(1);
    }

    private void commandPlaceDirection() {
        shoulderThrottle.setCurrentPosition(-1);
    }

    private void causeButtonToRiseForNextPeriodTask(FakeOnOffButton button) {
        button.setPressed(false);
        wobbleGoal.periodicTask();
        button.setPressed(true);
    }
}
