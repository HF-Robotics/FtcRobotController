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

package com.hfrobots.tnt.season2627.mechanisms;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.RunnableState;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.hfrobots.tnt.season2526.mechanisms.HoodController;
import com.hfrobots.tnt.season2526.vision.AprilTagAligner;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class WheeledLauncher implements PeriodicTask {

    private static final double LAUNCH_VELOCITY = 4000 ;

    private static double GATE_OPEN_POS = 1;
    private static double GATE_CLOSE_POS = 0;
    protected final DcMotorEx launcherMotor;

    protected final Servo gateServo;


    private double requestedVelocity = 0;

    private final Telemetry telemetry;

    public WheeledLauncher(final HardwareMap hardwareMap,
                           final Telemetry telemetry,
                           final Ticker ticker) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setVelocityPIDFCoefficients(135,3, 0, 14);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gateServo = hardwareMap.get (Servo.class,"gateServo");
        this.telemetry = telemetry;
    }

    public void adjustVelocity(final RangeInput adjustmentThrottle) {
        double adjustmentAmount = -adjustmentThrottle.getPosition();

        if (requestedVelocity < 0) {
            return;
        }

        if (adjustmentAmount != 0) {
            maybeSetLaunchVelocity(requestedVelocity + (adjustmentAmount * 4.0));
        }
    }

    public void openGate() {
        if (isAtTargetVelocity()) {
            gateServo.setPosition(GATE_OPEN_POS);
        }
    }

    public void closeGate(){
        gateServo.setPosition(GATE_CLOSE_POS);
    }

    public void launchVelocity() {
        maybeSetLaunchVelocity(LAUNCH_VELOCITY);
    }



    public void stopLauncher() {
        maybeSetLaunchVelocity(0);
    }

    public boolean isMoving() {
        return launcherMotor.getVelocity() > 50;
    }

    public boolean isAtTargetVelocity() {
        double currentVelocity = launcherMotor.getVelocity();

        return Math.abs(requestedVelocity - currentVelocity) < 100;
    }


    private void maybeSetLaunchVelocity(final double velocity) {
        if (requestedVelocity != velocity) {
            requestedVelocity = velocity;
            launcherMotor.setVelocity(requestedVelocity);
        }
    }

    @Override
    public void periodicTask() {
        String currentVelocityIndicator = (isAtTargetVelocity() ? "*" : "") + launcherMotor.getVelocity();
        telemetry.addData("Launcher",  "req_vel %s - cur_vel %s",
                Double.toString(requestedVelocity),
                currentVelocityIndicator);

    }

    class LauncherToSpeedState extends State {
        LauncherToSpeedState(final Telemetry telemetry) {
            super("Launcher to speed", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (isAtTargetVelocity()) {
                return nextState;
            }

            return this;
        }

        @Override
        public void resetToStart() {

        }
    }

    public void addLaunchStates(final SequenceOfStates sequenceOfStates) {
        final State startLauncher = new RunnableState("StartLauncher", telemetry,
                () -> launchVelocity());

        final LauncherToSpeedState toSpeedState = new LauncherToSpeedState(telemetry);

        final State dropPollenState = new RunnableState("Drop Pollen", telemetry,
                () -> openGate());

        final DelayState waitForLaunchState = new DelayState("waiting for launch", telemetry,
                5, TimeUnit.SECONDS);

        sequenceOfStates.addSequential(startLauncher);
        sequenceOfStates.addSequential(toSpeedState);
        sequenceOfStates.addSequential(dropPollenState);
        sequenceOfStates.addSequential(waitForLaunchState);
    }

}