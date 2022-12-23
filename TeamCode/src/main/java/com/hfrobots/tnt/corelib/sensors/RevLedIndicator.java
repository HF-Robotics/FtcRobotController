/*
 Copyright (c) 2022 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.corelib.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simplifies using a REV Led Indicator so that you don't
 * have to remember the various combinations of high/low
 * used for the two DigitalChannels for each color.
 */
public class RevLedIndicator {
    public enum LedColor {RED, GREEN, AMBER, OFF};

    private final DigitalChannel greenLed;

    private final DigitalChannel redLed;

    public RevLedIndicator(final HardwareMap hardwareMap,
                           final String namePrefix) {
        greenLed = hardwareMap.get(DigitalChannel.class, namePrefix + "Green");
        redLed = hardwareMap.get(DigitalChannel.class, namePrefix + "Red");

        greenLed.setMode(DigitalChannel.Mode.OUTPUT);
        redLed.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setColor(final LedColor color) {
        switch (color) {
            case GREEN:
                greenLed.setState(true);
                redLed.setState(false);
                break;
            case RED:
                greenLed.setState(false);
                redLed.setState(true);
                break;
            case AMBER:
                greenLed.setState(false);
                redLed.setState(false);
                break;
            case OFF:
                greenLed.setState(true);
                redLed.setState(true);
                break;
        }
    }
}
