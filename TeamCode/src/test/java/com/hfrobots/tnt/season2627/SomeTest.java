/*
 Copyright (c) 2026 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2627;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeCRServo;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.sensors.FakeDigitalChannel;
import com.google.common.testing.FakeTicker;
import com.hfrobots.tnt.fakes.FakeTelemetry;
import com.hfrobots.tnt.season2526.DecodeTestConstants;
import com.hfrobots.tnt.season2526.mechanisms.HoodController;
import com.hfrobots.tnt.season2526.mechanisms.WheeledLauncher;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.junit.Test;

public class SomeTest {
    private final FakeTelemetry telemetry = new FakeTelemetry();

    private final FakeTicker ticker;

    public SomeTest() {
        ticker = new FakeTicker();
    }

    @Test
    public void hardwareMapExists() {
        assertTrue(BioBuzzTestConstants.HARDWARE_MAP.size() > 0);
    }
}
