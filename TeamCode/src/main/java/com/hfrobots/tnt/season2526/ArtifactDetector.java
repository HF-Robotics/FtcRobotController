/*
 Copyright (c) 2025 The Tech Ninja Team (https://ftc9929.com)

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
package com.hfrobots.tnt.season2526;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import androidx.annotation.NonNull;

import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.hfrobots.tnt.util.LowPassFilter;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArtifactDetector implements PeriodicTask {
    final RevColorSensorV3 artifactColor1;

    final RevColorSensorV3 artifactColor2;

    final AnalogInput rightPresenceDetector;

    final AnalogInput leftPresenceDetector;

    final Servo colorIndicator;

    final Telemetry telemetry;

    private final LowPassFilter leftLowPassFilter = new LowPassFilter(0.78D);
    private final LowPassFilter rightLowPassFilter = new LowPassFilter(0.78D);

    static class RGBAD {
        final int red;

        final int green;

        final int blue;

        final int alpha;

        final double distanceMm;

        RGBAD(final RevColorSensorV3 colorSensor) {
            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();
            alpha = colorSensor.alpha();
            distanceMm = colorSensor.getDistance(DistanceUnit.MM);
        }


        @NonNull
        @Override
        public String toString() {
            return red + "," + green + ", " + blue;
        }
    }

    public ArtifactDetector(final HardwareMap hardwareMap, Telemetry telemetry) {
        artifactColor1 = hardwareMap.get(RevColorSensorV3.class, "artifactColor1");

        artifactColor2 = hardwareMap.get(RevColorSensorV3.class, "artifactColor2");

        colorIndicator = hardwareMap.get(Servo.class, "ledIndicatorBack");

        leftPresenceDetector = hardwareMap.get(AnalogInput.class, "leftPresenceDetector");

        rightPresenceDetector = hardwareMap.get(AnalogInput.class, "rightPresenceDetector");

        this.telemetry = telemetry;
    }

    @Override
    public void periodicTask() {
        // Only read if the robot detects a possible artifact present

        double leftVoltage = leftPresenceDetector.getVoltage();

        double rightVoltage = rightPresenceDetector.getVoltage();

        leftVoltage = leftLowPassFilter.filter(leftVoltage);

        rightVoltage = rightLowPassFilter.filter(rightVoltage);

        if (leftVoltage < 0.5 && rightVoltage < 0.5) {
            setPresenceSignal(true);
            colorDetectionLogic();
        } else {
            setPresenceSignal(false);
        }

        colorDetectionLogic();
    }

    private void setPresenceSignal(final boolean isPresent) {
        if (isPresent) {
            colorIndicator.setPosition(DecodeDriveTeamSignal.AZURE_LED);
        } else {
            colorIndicator.setPosition(0);
        }
    }

    private void colorDetectionLogic() {

        // Detect RGB from each color sensor, they read as
        // individual values for red, green, blue

        // Make a decision, is it green, purple or unknown
        // and set the RGB indicator to show the drive team

        // Don't fail the robot code if I2C readings fail

        try {
            RGBAD colorValues1 = new RGBAD(artifactColor1);
            RGBAD colorValues2 = new RGBAD(artifactColor2);

            telemetry.addData("Artifact", colorValues1 + " | " + colorValues2);

            if (isPurple(colorValues1) ||
                isPurple(colorValues2)) {
                colorIndicator.setPosition(DecodeDriveTeamSignal.VIOLET_LED);
            } else if (isGreen(colorValues1) ||
                isGreen(colorValues2)) {
                colorIndicator.setPosition(DecodeDriveTeamSignal.GREEN_LED);
            } else {
                // It's unknown. turn off the RGB indicator
                //colorIndicator.setPosition(0);
            }
        } catch (Exception ex) {
            Log.e(LOG_TAG, "Failed to read from color sensors", ex);

            colorIndicator.setPosition(0);
        }
    }

    private boolean isPurple(final RGBAD colorValues) {
        if (!isGoodReading(colorValues)) {
            return false;
        }

        return colorValues.blue > colorValues.green;
    }

    private boolean isGreen(final RGBAD colorValues) {
        if (!isGoodReading(colorValues)) {
            return false;
        }

        return colorValues.green > colorValues.red /*&& green > blue*/;
    }

    private boolean isGoodReading(final RGBAD colorValues) {
        return colorValues.red > 1000 && colorValues.green > 1000 && colorValues.blue > 1000;
    }
}
