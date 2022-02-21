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

import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

/**
 * The class that makes the CarouselMechanism (to deliver ducks and the team scoring element)
 * work.
 */
public class CarouselMechanism {
    private static final int NON_FLIGHT_VELOCITY = 750;

    private static final int AUTO_DUCK_FLIGHT_VELOCITY = 500;
    private static final int BLUE_MAGNITUDE = -1;
    private static final int RED_MAGNITUDE = 1;

    private static final double CURVE_DURATION_MS = 1500;

    private static final int START_VELOCITY = 550;

    private static final int MAX_VELOCITY = 900;

    private static final double ACCELERATION_VEL_MS = (MAX_VELOCITY - START_VELOCITY) / CURVE_DURATION_MS;

    private int currentVelocity = START_VELOCITY;

    private Stopwatch stopwatch = Stopwatch.createUnstarted();

    // (1) We need to add the motors, servos and sensors this mechanism will use first, they go
    // in this location in the file. The mechanism requirements document can be consulted to
    // figure out what these are.

    private final DcMotorEx carouselMotor;

    public CarouselMechanism(@NonNull final HardwareMap hardwareMap) {

        // (2) Here is where we setup the items in (1), by finding them in the HardwareMap
        //
        // If this mechanism will have its own automation through a state machine,
        // this would also be setup here.
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
    }

    // (3) Here, we define what the mechanism does, by adding methods. These methods contain
    // instructions for what to do with the items listed in step (1).

    public void spinBlue() {
        computeVelocity();
        carouselMotor.setPower(BLUE_MAGNITUDE);
        carouselMotor.setVelocity(BLUE_MAGNITUDE * currentVelocity);
    }

    public void spinRed() {
        computeVelocity();
        carouselMotor.setPower(RED_MAGNITUDE);
        carouselMotor.setVelocity(RED_MAGNITUDE * currentVelocity);
    }

    private void computeVelocity() {
        if (!stopwatch.isRunning()) {
            stopwatch.start();
        } else {
            long elapsedMs = Math.max(1, stopwatch.elapsed(TimeUnit.MILLISECONDS));

            int addToVelocity = (int)(ACCELERATION_VEL_MS * elapsedMs);
            currentVelocity = currentVelocity + addToVelocity;

            if (currentVelocity > MAX_VELOCITY) {
                currentVelocity = MAX_VELOCITY;
            }

            stopwatch.reset();
            stopwatch.start();

        }
    }

    public void spinBlueForAuto() {
        carouselMotor.setPower(BLUE_MAGNITUDE);
        carouselMotor.setVelocity(BLUE_MAGNITUDE * AUTO_DUCK_FLIGHT_VELOCITY);
    }

    public void spinRedForAuto() {
        carouselMotor.setPower(RED_MAGNITUDE);
        carouselMotor.setVelocity(RED_MAGNITUDE * AUTO_DUCK_FLIGHT_VELOCITY);

    }

    public void stop() {
        currentVelocity = START_VELOCITY;

        if (stopwatch.isRunning()) {
            stopwatch.stop();
        }

        carouselMotor.setPower(0);
        carouselMotor.setVelocity(0);
    }


    // (4) If this component needs to take action during every loop of tele-op or auto,
    // implement the PeriodicTask interface and its required methods, and write the
    // code that performs those actions in the periodicTask() method.

    // (5) If this mechanism will have automation using a state machine, we describe the
    // states as new classes here, and set them up in (2).
}
