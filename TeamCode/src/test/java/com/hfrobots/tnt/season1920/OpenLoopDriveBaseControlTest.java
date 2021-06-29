/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
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
 **/

package com.hfrobots.tnt.season1920;

import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.util.FakeHardwareMapFactory;
import com.hfrobots.tnt.corelib.drive.roadrunner.RoadRunnerMecanumDrive;
import com.hfrobots.tnt.fakes.sensors.FakeBNO055IMU;
import com.hfrobots.tnt.fakes.FakeHardwareMap;
import com.hfrobots.tnt.season2021.UltimateGoalTestConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class OpenLoopDriveBaseControlTest {
    private HardwareMap hardwareMap;

    private FakeDcMotorEx leftFrontDriveMotor;

    private FakeDcMotorEx leftRearDriveMotor;

    private FakeDcMotorEx rightFrontDriveMotor;

    private FakeDcMotorEx rightRearDriveMotor;

    private FakeBNO055IMU imu;

    private RoadRunnerMecanumDrive drivebase;

    protected FakeRangeInput leftStickY;

    protected FakeRangeInput leftStickX;

    protected FakeRangeInput rightStickX;

    protected RangeInput driveInvertedButton;

    protected RangeInput driveFastButton;

    protected OnOffButton driveBumpStrafeRightButton;

    protected OnOffButton driveBumpStrafeLeftButton;

    protected DriverControls controls;

    @Before
    public void setUp() {
        hardwareMap = FakeHardwareMapFactory.getFakeHardwareMap("skystonev2.xml");
        leftFrontDriveMotor = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor");
        leftRearDriveMotor = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor");
        rightFrontDriveMotor = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor");
        rightRearDriveMotor = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "rightRearDriveMotor");

        hardwareMap.put("imu", new FakeBNO055IMU());

        hardwareMap.voltageSensor.put("voltage", new VoltageSensor() {
            @Override
            public double getVoltage() {
                return 12;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return null;
            }

            @Override
            public String getConnectionInfo() {
                return null;
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        });

        RoadRunnerMecanumDrive drivebase = new RoadRunnerMecanumDrive(hardwareMap);

        OpenLoopMecanumKinematics kinematics = new OpenLoopMecanumKinematics(drivebase);

        // FIXME: Create the driver controls

        // First, you need to create fake versions of all of the buttons above
        leftStickY = new FakeRangeInput();

        leftStickX = new FakeRangeInput();

        rightStickX = new FakeRangeInput();

        driveInvertedButton = new FakeRangeInput();

        driveFastButton = new FakeRangeInput();

        driveBumpStrafeRightButton = new FakeOnOffButton();

        driveBumpStrafeLeftButton = new FakeOnOffButton();

        controls = DriverControls.builder()
                .kinematics(kinematics)
                .leftStickY(leftStickY)
                .leftStickX(leftStickX)
                .rightStickX(rightStickX)
                .rightTrigger(driveInvertedButton)
                .leftTrigger(driveFastButton)
                .rightBumper(driveBumpStrafeRightButton)
                .leftBumper(driveBumpStrafeLeftButton).build();
    }

    @Test
    public void openLoopControl() {
       // drive straight
        leftStickY.setCurrentPosition(-1);
        controls.periodicTask();

        double leftFrontPower = leftFrontDriveMotor.getPower();
        double leftRearPower = leftRearDriveMotor.getPower();
        double rightFrontPower = rightFrontDriveMotor.getPower();
        double rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertEquals(leftFrontPower, leftRearPower, .01);
        Assert.assertEquals(rightFrontPower, rightRearPower, .01);


        // stop all movement or standstill
        leftStickY.setCurrentPosition(0);
        leftStickX.setCurrentPosition(0);
        rightStickX.setCurrentPosition(0);
        controls.periodicTask();

        leftFrontPower = leftFrontDriveMotor.getPower();
        leftRearPower = leftRearDriveMotor.getPower();
        rightFrontPower = rightFrontDriveMotor.getPower();
        rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertEquals(0, leftFrontPower, .02);
        Assert.assertEquals(0, leftRearPower, .02);
        Assert.assertEquals(0, rightFrontPower, .02);
        Assert.assertEquals(0, rightRearPower, .02);

        leftStickY.setCurrentPosition(0);
        leftStickX.setCurrentPosition(0);
        rightStickX.setCurrentPosition(-1);
        controls.periodicTask();

        leftFrontPower = leftFrontDriveMotor.getPower();
        leftRearPower = leftRearDriveMotor.getPower();
        rightFrontPower = rightFrontDriveMotor.getPower();
        rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertTrue(leftFrontPower < 0);
        Assert.assertTrue(leftRearPower < 0);
        Assert.assertTrue(rightFrontPower > 0);
        Assert.assertTrue(rightRearPower > 0);

        leftStickY.setCurrentPosition(0);
        leftStickX.setCurrentPosition(-1);
        rightStickX.setCurrentPosition(0);
        controls.periodicTask();

        leftFrontPower = leftFrontDriveMotor.getPower();
        leftRearPower = leftRearDriveMotor.getPower();
        rightFrontPower = rightFrontDriveMotor.getPower();
        rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertTrue(leftFrontPower < 0);
        Assert.assertTrue(leftRearPower > 0);
        Assert.assertTrue(rightFrontPower > 0);
        Assert.assertTrue(rightRearPower < 0);

    }
}
