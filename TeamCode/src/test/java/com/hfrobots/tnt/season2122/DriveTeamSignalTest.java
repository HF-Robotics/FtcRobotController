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

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.common.testing.FakeTicker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class DriveTeamSignalTest {

    private DriveTeamSignal driveTeamSignal;

    private FakeTicker fakeTicker = new FakeTicker();

    @Before
    public void setup() {
        final HardwareMap hardwareMap = FreightFrenzyTestConstants.HARDWARE_MAP;
        driveTeamSignal = new DriveTeamSignal(hardwareMap, fakeTicker);
    }

    @Test
    public void matchStages() {
        driveTeamSignal.startMatch();

        assertTrue(driveTeamSignal.matchStarted());

        fakeTicker.advance(81, TimeUnit.SECONDS);

        assertTrue(driveTeamSignal.readyForEndGame());
        assertFalse(driveTeamSignal.startWarehouseRun());
        assertFalse(driveTeamSignal.isEndGame());
    }

}
