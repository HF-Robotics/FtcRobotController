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

package com.hfrobots.tnt.season2122;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.ftc9929.corelib.control.ToggledButton;
import com.ftc9929.testing.fakes.FakeTelemetry;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Before;
import org.junit.Test;

public class FreightManipulatorTest {

    private FreightManipulator freightManipulator;

    private FakeTelemetry fakeTelemetry = new FakeTelemetry();

    private FakeRangeInput liftThrottle = new FakeRangeInput();

    private FakeOnOffButton unsafeButton = new FakeOnOffButton();

    private FakeOnOffButton toHubLevelOne = new FakeOnOffButton();

    private FakeOnOffButton toHubLevelTwo = new FakeOnOffButton();

    private FakeOnOffButton toHubLevelThree = new FakeOnOffButton();

    private FakeOnOffButton gripper = new FakeOnOffButton();

    private FakeDcMotorEx armMotorWithPid;

    @Before
    public void setup() {
        final HardwareMap hardwareMap = FreightFrenzyTestConstants.HARDWARE_MAP;

        armMotorWithPid = (FakeDcMotorEx)hardwareMap.get(DcMotorEx.class, "armMotor");

        freightManipulator = new FreightManipulator(hardwareMap, fakeTelemetry);
        freightManipulator.setUnsafeButton(unsafeButton);
        freightManipulator.setArmThrottle(liftThrottle);
        freightManipulator.setToHubLevelOneButton(toHubLevelOne.debounced());
        freightManipulator.setToHubLevelTwoButton(toHubLevelTwo.debounced());
        freightManipulator.setToHubLevelThreeButton(toHubLevelThree.debounced());
        freightManipulator.setGripperToggleButton(new ToggledButton(gripper));
    }

    @Test
    public void happyPathStates() {
        freightManipulator.periodicTask();
        assertEquals("Lift Idle", freightManipulator.getCurrentStateName());

        liftThrottleForward();
        freightManipulator.periodicTask();
        assertEquals("Lift - OL - Rising", freightManipulator.getCurrentStateName());

        liftThrottleCenter();
        freightManipulator.periodicTask();
        assertEquals("Lift Idle", freightManipulator.getCurrentStateName());

        liftThrottleForward();
        freightManipulator.periodicTask();
        assertEquals("Lift - OL - Rising", freightManipulator.getCurrentStateName());

        liftThrottleBack();
        freightManipulator.periodicTask();
        assertEquals("Lift Idle", freightManipulator.getCurrentStateName());

        freightManipulator.periodicTask();
        assertEquals("Lift - OL - Lowering", freightManipulator.getCurrentStateName());

        liftThrottleCenter();
        freightManipulator.periodicTask();

        assertAutoLevel(toHubLevelOne,"Lift - CL - One");
        assertAutoLevel(toHubLevelTwo, "Lift - CL - Two");
        assertAutoLevel(toHubLevelThree, "Lift - CL - Three");
    }

    private void assertAutoLevel(final FakeOnOffButton toLevelButton, final String expectedStateName) {
        buttonPress(toLevelButton);
        assertEquals(expectedStateName, freightManipulator.getCurrentStateName());
        freightManipulator.periodicTask();
        assertTrue(armMotorWithPid.isBusy());
        armMotorWithPid.setBusy(false); // pretend it reached position
        freightManipulator.periodicTask();
        assertEquals("Lift Idle", freightManipulator.getCurrentStateName());

        buttonPress(toLevelButton);
        assertEquals(expectedStateName, freightManipulator.getCurrentStateName());
        freightManipulator.periodicTask();
        assertTrue(armMotorWithPid.isBusy());

        liftThrottleForward();
        freightManipulator.periodicTask();
        assertEquals("Lift Idle", freightManipulator.getCurrentStateName());

        freightManipulator.periodicTask();
        liftThrottleCenter();
        freightManipulator.periodicTask();
        assertEquals("Lift Idle", freightManipulator.getCurrentStateName());
    }

    private void liftThrottleCenter() {
        liftThrottle.setCurrentPosition(0.0f);
    }

    private void liftThrottleForward() {
        liftThrottle.setCurrentPosition(-0.5f);
    }

    private void liftThrottleBack() {
        liftThrottle.setCurrentPosition(0.5f);
    }

    private void buttonPress(final FakeOnOffButton button) {
        button.setPressed(false);
        freightManipulator.periodicTask();
        button.setPressed(true);
        freightManipulator.periodicTask();
    }
}
