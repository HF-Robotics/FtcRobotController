/**
 Copyright (c) 2016, 2017, HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.util;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * An OpMode that allows you to test any/all of the sensors on a robot
 *
 * gamepad1.left_bumper = cycle through sensor types
 * gamepad1.right_bumper = cycle through all configured sensors of current type
 *
 * current sensor type, name and data are displayed in driver station telemetry
 *
 */
@TeleOp(name="Sensor Tester", group="Utilities")
@SuppressWarnings("unused")
public class SensorTester extends OpMode {
    private List<NamedDeviceMap.NamedDevice<Rev2mDistanceSensor>> namedTofSensors;

    private List<NamedDeviceMap.NamedDevice<TouchSensor>> namedTouchDevices;

    private List<NamedDeviceMap.NamedDevice<LynxI2cColorRangeSensor>> namedLynxColorSensors;

    private List<NamedDeviceMap.NamedDevice<RevColorSensorV3>> namedRevV3ColorSensors;

    private List<NamedDeviceMap.NamedDevice<DigitalChannel>> namedDigitalChannels;

    private List<NamedDeviceMap.NamedDevice<AnalogInput>> namedAnalogInputs;

    private List<NamedDeviceMap.NamedDevice<IMU>> namedImus;

    private int currentListPosition;

    private DebouncedButton leftBumper;
    private DebouncedButton rightBumper;

    private DebouncedButton aButton;

    private NamedDeviceMap namedDeviceMap;

    @Override
    public void init() {
        namedDeviceMap = new NamedDeviceMap(hardwareMap);

        namedDigitalChannels = namedDeviceMap.getAll(DigitalChannel.class);
        namedLynxColorSensors = namedDeviceMap.getAll(LynxI2cColorRangeSensor.class);
        namedRevV3ColorSensors = namedDeviceMap.getAll(RevColorSensorV3.class);
        namedTouchDevices = namedDeviceMap.getAll(TouchSensor.class);
        namedImus = namedDeviceMap.getAll(IMU.class);
        namedTofSensors = namedDeviceMap.getAll(Rev2mDistanceSensor.class);
        namedAnalogInputs = namedDeviceMap.getAll(AnalogInput.class);

        initializeLynxEmbeddedImus();

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        leftBumper = new DebouncedButton(ninjaGamePad.getLeftBumper());
        rightBumper = new DebouncedButton(ninjaGamePad.getRightBumper());
        aButton = new DebouncedButton(ninjaGamePad.getAButton());
    }

    protected void initializeLynxEmbeddedImus() {
        for (NamedDeviceMap.NamedDevice<IMU> namedImu : namedImus) {
            namedImu.getDevice().initialize(
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                            )
                    )
            );
        }
    }

    private boolean ledEnabled = false;

    private enum Mode { LYNX_COLOR, REV_COLOR_V3, LYNX_IMU, TOF_SENSOR,  DIGITAL_CHANNEL, ANALOG_INPUT, TOUCH }

    private Mode currentMode = Mode.LYNX_COLOR;

    @Override
    public void loop() {
        if (leftBumper.getRise()) {
            switch (currentMode) {
                case LYNX_COLOR:
                    currentMode = Mode.REV_COLOR_V3;
                    break;
                case REV_COLOR_V3:
                    currentMode = Mode.LYNX_IMU;
                    break;
                case LYNX_IMU:
                    currentMode = Mode.TOF_SENSOR;
                    break;
                case TOF_SENSOR:
                    currentMode = Mode.DIGITAL_CHANNEL;
                    break;
                case DIGITAL_CHANNEL:
                    currentMode = Mode.ANALOG_INPUT;
                    break;
                case ANALOG_INPUT:
                    currentMode = Mode.TOUCH;
                    break;
                case TOUCH:
                    currentMode = Mode.LYNX_COLOR;
                    break;
            }
        }

        switch (currentMode) {
            case LYNX_COLOR:
                doLynxColorSensorLoop();
                break;
            case LYNX_IMU:
                doLynxImuLoop();
                break;
            case TOF_SENSOR:
                doTofSensorLoop();
                break;
            case DIGITAL_CHANNEL:
                doDigitalChannelLoop();
                break;
            case ANALOG_INPUT:
                doAnalogInputLoop();
                break;
            case TOUCH:
                doTouchSensorLoop();
                break;
        }
    }

    private int currentTofSensorListPosition = 0;

    private void doTofSensorLoop() {
        namedTofSensors = namedDeviceMap.getAll(Rev2mDistanceSensor.class);

        if (namedTofSensors.isEmpty()) {
            telemetry.addData("No TOF sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentTofSensorListPosition++;

            if (currentTofSensorListPosition == namedTofSensors.size()) {
                currentTofSensorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<Rev2mDistanceSensor> currentNamedTofSensor = namedTofSensors.get(
                currentTofSensorListPosition);
        Rev2mDistanceSensor currentTofSensor = currentNamedTofSensor.getDevice();
        String sensorName = currentNamedTofSensor.getName();
        telemetry.addData("TOF ",  "%s", sensorName);
        telemetry.addData("Range (in)", "%f", currentTofSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Is Timed out?", "%s", Boolean.toString(currentTofSensor.didTimeoutOccur()));
        updateTelemetry(telemetry);
    }

    int currentLynxColorListPosition = 0;

    private void doLynxColorSensorLoop() {
        namedLynxColorSensors = namedDeviceMap.getAll(LynxI2cColorRangeSensor.class);

        if (namedLynxColorSensors.isEmpty()) {
            telemetry.addData("No lynx i2c color sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentLynxColorListPosition++;

            if (currentLynxColorListPosition == namedLynxColorSensors.size()) {
                currentLynxColorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<LynxI2cColorRangeSensor> currentNamedLynxColorSensor = namedLynxColorSensors.get(currentLynxColorListPosition);
        LynxI2cColorRangeSensor currentColorSensor = currentNamedLynxColorSensor.getDevice();
        String sensorName = currentNamedLynxColorSensor.getName();

        if (aButton.getRise()) {
            ledEnabled = !ledEnabled;
        }

        currentColorSensor.enableLed(ledEnabled);
        int alpha = currentColorSensor.alpha();
        int red = currentColorSensor.red();
        int green = currentColorSensor.green();
        int blue = currentColorSensor.blue();

        telemetry.addData("color ",  "%s x blk cal, y wt cal", sensorName);
        telemetry.addData("R G B A", "%d %d %d %d", red, green, blue, alpha);
        telemetry.addData("D (mm):", "%f", currentColorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("light:", "%f", currentColorSensor.getLightDetected());
        telemetry.addData("raw light:", "%f", currentColorSensor.getRawLightDetected());
        updateTelemetry(telemetry);
    }

    int currentRevColorV3ListPosition = 0;

    private void doRevColorSensorV3Loop() {
        if (namedRevV3ColorSensors.isEmpty()) {
            telemetry.addData("No REV V3 i2c color sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentRevColorV3ListPosition++;

            if (currentRevColorV3ListPosition == namedRevV3ColorSensors.size()) {
                currentRevColorV3ListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<RevColorSensorV3> currentNamedRevColorSensorV3 = namedRevV3ColorSensors.get(currentRevColorV3ListPosition);
        RevColorSensorV3 currentColorSensor = currentNamedRevColorSensorV3.getDevice();
        String sensorName = currentNamedRevColorSensorV3.getName();

        if (aButton.getRise()) {
            ledEnabled = !ledEnabled;
        }

        currentColorSensor.enableLed(ledEnabled);
        int alpha = currentColorSensor.alpha();
        int red = currentColorSensor.red();
        int green = currentColorSensor.green();
        int blue = currentColorSensor.blue();

        telemetry.addData("color ",  "%s x blk cal, y wt cal", sensorName);
        telemetry.addData("R G B A", "%d %d %d %d", red, green, blue, alpha);
        telemetry.addData("D (mm):", "%f", currentColorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("light:", "%f", currentColorSensor.getLightDetected());
        telemetry.addData("raw light:", "%f", currentColorSensor.getRawLightDetected());
        updateTelemetry(telemetry);
    }

    int currentLynxImuListPosition = 0;

    private void doLynxImuLoop() {
        namedImus = namedDeviceMap.getAll(IMU.class);

        if (namedImus.isEmpty()) {
            telemetry.addData("No lynx imus", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentLynxImuListPosition++;

            if (currentLynxImuListPosition == namedImus.size()) {
                currentLynxImuListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<IMU> currentNamedLynxImu = namedImus.get(currentLynxImuListPosition);
        IMU currentImu = currentNamedLynxImu.getDevice();
        String sensorName = currentNamedLynxImu.getName();

        YawPitchRollAngles currentOrientation = currentImu.getRobotYawPitchRollAngles();

        // First angle is heading, second is roll, third is pitch
        double heading = currentOrientation.getYaw(AngleUnit.DEGREES);
        double roll = currentOrientation.getRoll(AngleUnit.DEGREES);
        double pitch = currentOrientation.getPitch(AngleUnit.DEGREES);

        telemetry.addData("imu ",  "%s", sensorName);
        telemetry.addData("h/r/p ",  "%.2f %.2f %.2f", heading, roll, pitch);

        updateTelemetry(telemetry);
    }

    int currentDigitalChannelListPosition = 0;

    private void doDigitalChannelLoop() {
        namedLynxColorSensors = namedDeviceMap.getAll(LynxI2cColorRangeSensor.class);

        if (namedDigitalChannels.isEmpty()) {
            telemetry.addData("No digital channels", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentDigitalChannelListPosition++;

            if (currentDigitalChannelListPosition == namedDigitalChannels.size()) {
                currentDigitalChannelListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<DigitalChannel> currentNamedDigitalChannel = namedDigitalChannels.get(currentDigitalChannelListPosition);
        DigitalChannel currentDigitalChannel = currentNamedDigitalChannel.getDevice();
        currentDigitalChannel.setMode(DigitalChannel.Mode.INPUT);
        String sensorName = currentNamedDigitalChannel.getName();

        telemetry.addData("digital ",  "%s = %s", sensorName, String.valueOf(currentDigitalChannel.getState()));

        updateTelemetry(telemetry);
    }

    int currentAnalogInputListPosition = 0;

    private void doAnalogInputLoop() {
        namedAnalogInputs = namedDeviceMap.getAll(AnalogInput.class);

        if (namedAnalogInputs.isEmpty()) {
            telemetry.addData("No analog inputs", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentAnalogInputListPosition++;

            if (currentAnalogInputListPosition == namedAnalogInputs.size()) {
                currentAnalogInputListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<AnalogInput> currentNamedAnalogInput = namedAnalogInputs.get(currentAnalogInputListPosition);
        AnalogInput currentAnalogInput = currentNamedAnalogInput.getDevice();
        String sensorName = currentNamedAnalogInput.getName();

        telemetry.addData("analog ",  "%s = %s", sensorName, String.valueOf(currentAnalogInput.getVoltage()));

        updateTelemetry(telemetry);
    }

    private int currentTouchSensorListPosition = 0;

    private void doTouchSensorLoop() {
        if (namedTouchDevices.isEmpty()) {
            telemetry.addData("No touch sensors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentTouchSensorListPosition++;

            if (currentTouchSensorListPosition == namedTouchDevices.size()) {
                currentTouchSensorListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<TouchSensor> currentNamedTouchSensor = namedTouchDevices.get(currentTouchSensorListPosition);
        TouchSensor currentTouchSensor = currentNamedTouchSensor.getDevice();
        String sensorName = currentNamedTouchSensor.getName();
        boolean isPressed = currentTouchSensor.isPressed();
        telemetry.addData("touch ",  "%s", sensorName);
        telemetry.addData("is pressed", "%s", Boolean.toString(isPressed));
        updateTelemetry(telemetry);
    }
}
