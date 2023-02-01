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

package com.hfrobots.tnt.season2223;

import com.qualcomm.robotcore.hardware.Servo;

import lombok.NonNull;

public class Gripper {

    private final Servo servo;

    private static final double OPEN_POSITION = 0.586;

    private static final double CLOSED_POSITION_GOBILDA_TORQUE = .9;

    private static final double CLOSED_POSITION_REV = 1.0;

    private static final double CLOSED_POSITION = CLOSED_POSITION_REV;

    public Gripper(@NonNull Servo servo) {
        this.servo = servo;
        servo.setPosition(OPEN_POSITION);
    }

    public void open() {
        servo.setPosition(OPEN_POSITION);
    }

    public void close() {
        servo.setPosition(CLOSED_POSITION);
    }

    public boolean isOpen() {
        return Math.abs(servo.getPosition() - OPEN_POSITION) < 0.1;
    }

    public boolean isClosed() {
        return !isOpen();
    }
}
