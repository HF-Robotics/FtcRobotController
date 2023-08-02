/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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

import static org.powermock.api.mockito.PowerMockito.when;

import android.os.Environment;

import androidx.annotation.Nullable;

import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.google.common.testing.FakeTicker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.junit.runner.RunWith;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

import java.io.IOException;
import java.util.List;

@RunWith(PowerMockRunner.class)
@PrepareForTest(Environment.class)
public class IntakeTest {
    private HardwareMap hwMap;

    private FakeDcMotorEx intakeMotor;

    @Rule
    TemporaryFolder tempFolder = new TemporaryFolder();

    @Before
    public void setup() throws IOException {
        PowerMockito.mockStatic(Environment.class);

        when(Environment.getExternalStorageDirectory()).thenReturn(tempFolder.newFolder());

        hwMap = new HardwareMap(null, new OpModeManagerNotifier() {
            @Override
            public OpMode registerListener(Notifications listener) {
                return null;
            }

            @Override
            public void unregisterListener(Notifications listener) {

            }
        }) {
            @Nullable
            @Override
            public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
                synchronized (lock) {
                    deviceName = deviceName.trim();
                    List<HardwareDevice> list = allDevicesMap.get(deviceName);
                    @Nullable T result = null;

                    if (list != null) {
                        for (HardwareDevice device : list) {
                            if (classOrInterface.isInstance(device)) {
                                result = classOrInterface.cast(device);
                                break;
                            }
                        }
                    }

                    return result;
                }
            }
        };

        intakeMotor = new FakeDcMotorEx();

        hwMap.put("intakeMotor", intakeMotor);
    }

    @Test
    public void testIntake() {
        Intake intakeToTest = new Intake(hwMap, new FakeTicker());
        intakeToTest.intake(Intake.INTAKE_POWER);

        Assert.assertEquals(Intake.INTAKE_POWER, intakeMotor.getPower(), 0.01);

        intakeToTest.outtake(Intake.INTAKE_POWER);

        Assert.assertEquals(Intake.OUTTAKE_POWER, intakeMotor.getPower(), 0.01);
    }
}
