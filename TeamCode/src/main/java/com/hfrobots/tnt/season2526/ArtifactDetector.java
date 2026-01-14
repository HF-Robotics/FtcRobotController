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

import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArtifactDetector implements PeriodicTask {
    final RevColorSensorV3 artifactColor1;

    final RevColorSensorV3 artifactColor2;

    final Servo colorIndicator;

    final Telemetry telemetry;

    private int color1red;
    private int color1green;
    private int color1blue;
    private int color2red;
    private int color2green;
    private int color2blue;
    public ArtifactDetector(final HardwareMap hardwareMap, Telemetry telemetry) {
        artifactColor1 = hardwareMap.get(RevColorSensorV3.class, "artifactColor1");

        artifactColor2 = hardwareMap.get(RevColorSensorV3.class, "artifactColor2");

        colorIndicator = hardwareMap.get(Servo.class, "ledIndicatorBack");

        this.telemetry = telemetry;
    }


    @Override
    public void periodicTask() {
        // Detect RGB from each color sensor, they read as
        // individual values for red, green, blue

        // Make a decision, is it green, purple or unknown
        // and set the RGB indicator to show the drive team

        color1red = artifactColor1.red();
        color1green = artifactColor1.green();
        color1blue = artifactColor1.blue();

        color2red = artifactColor2.red();
        color2green = artifactColor2.green();
        color2blue = artifactColor2.blue();

        if ((color1red > color1green && color1blue > color1green) || (color2red > color2green && color2blue > color2green)) {
            // it's purple
            colorIndicator.setPosition(DecodeDriveTeamSignal.VIOLET_LED);
        } else if ((color1green > color1red && color1green > color1blue) && (color2green > color2red && color2green > color2blue)) {
            // it's green
            colorIndicator.setPosition(DecodeDriveTeamSignal.GREEN_LED);
        } else {
            // It's unknown. turn off the RGB indicator
            colorIndicator.setPosition(0);
        }

        telemetry.addData("Art", "%d %d %d | %d %d %d",
                color1red,
                color1green,
                color1blue,
                color2red,
                color2green,
                color2blue);

    }
}
