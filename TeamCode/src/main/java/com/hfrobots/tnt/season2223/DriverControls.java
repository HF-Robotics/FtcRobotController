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

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.LowPassFilteredRangeInput;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.ParametricScaledRangeInput;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.ftc9929.corelib.drive.OpenLoopMecanumKinematics;
import com.hfrobots.tnt.corelib.task.PeriodicTask;

import lombok.Builder;

public class DriverControls implements PeriodicTask {
    protected RangeInput leftStickX;

    protected RangeInput leftStickY;

    protected RangeInput rightStickX;

    protected RangeInput rightStickY;

    protected DebouncedButton dpadUp;

    protected DebouncedButton dpadDown;

    protected DebouncedButton dpadLeft;

    protected DebouncedButton dpadRight;

    protected OnOffButton xBlueButton;

    protected DebouncedButton bRedButton;

    protected DebouncedButton yYellowButton;

    protected DebouncedButton aGreenButton;

    protected OnOffButton rightBumper;

    protected OnOffButton leftBumper;

    protected RangeInput leftTrigger;

    protected RangeInput rightTrigger;

    // Derived
    protected RangeInput driveForwardReverse;

    protected RangeInput driveStrafe;

    protected RangeInput driveRotate;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected OnOffButton driveInvertedButton;

    protected OnOffButton driveFastButton;

    protected OnOffButton strafePriorityButton;

    protected OnOffButton fineStrafeLeft;

    protected OnOffButton fineStrafeRight;

    private NinjaGamePad driversGamepad;

    private OpenLoopMecanumKinematics kinematics;

    private LiftMechanism liftMechanism;

    private final float throttleGain = 0.7F;

    private final float throttleExponent = 11; // MUST BE AN ODD NUMBER!

    private final float throttleDeadband = 0;

    private final float lowPassFilterFactor = 1.0F;

    @Builder
    private DriverControls(RangeInput leftStickX,
                           RangeInput leftStickY,
                           RangeInput rightStickX,
                           RangeInput rightStickY,
                           DebouncedButton dpadUp,
                           DebouncedButton dpadDown,
                           DebouncedButton dpadLeft,
                           DebouncedButton dpadRight,
                           OnOffButton dpadUpRaw,
                           OnOffButton dpadDownRaw,
                           OnOffButton dpadLeftRaw,
                           OnOffButton dpadRightRaw,
                           OnOffButton xBlueButton,
                           DebouncedButton bRedButton,
                           DebouncedButton yYellowButton,
                           DebouncedButton aGreenButton,
                           OnOffButton rightBumper,
                           OnOffButton leftBumper,
                           RangeInput leftTrigger,
                           RangeInput rightTrigger,
                           NinjaGamePad driversGamepad,
                           OpenLoopMecanumKinematics kinematics,
                           LiftMechanism liftMechanism) {
        if (driversGamepad != null) {
            this.driversGamepad = driversGamepad;
            setupFromGamepad();
        } else {
            this.leftStickX = leftStickX;
            this.leftStickY = leftStickY;
            this.rightStickX = rightStickX;
            this.rightStickY = rightStickY;
            this.dpadUp = dpadUp;
            this.dpadDown = dpadDown;
            this.dpadLeft = dpadLeft;
            this.dpadRight = dpadRight;
            this.xBlueButton = xBlueButton;
            this.bRedButton = bRedButton;
            this.yYellowButton = yYellowButton;
            this.aGreenButton = aGreenButton;
            this.rightBumper = rightBumper;
            this.leftBumper = leftBumper;
            this.leftTrigger = leftTrigger;
            this.rightTrigger = rightTrigger;
        }

        setupDerivedControls();
        setupCurvesAndFilters();

        this.liftMechanism = liftMechanism;
        this.kinematics = kinematics;
    }

    private void setupCurvesAndFilters() {
        driveStrafe = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(leftStickX, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveForwardReverse = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(leftStickY, lowPassFilterFactor),
                throttleDeadband, throttleGain, throttleExponent);

        driveRotate = new ParametricScaledRangeInput(
                new LowPassFilteredRangeInput(rightStickX, lowPassFilterFactor),
                throttleDeadband, 0.4F, 9);
    }

    private void setupFromGamepad() {
        leftStickX = driversGamepad.getLeftStickX();
        leftStickY = driversGamepad.getLeftStickY();
        rightStickX = driversGamepad.getRightStickX();
        rightStickY = driversGamepad.getRightStickY();

        dpadDown = new DebouncedButton(driversGamepad.getDpadDown());
        dpadUp = new DebouncedButton(driversGamepad.getDpadUp());
        dpadLeft = new DebouncedButton(driversGamepad.getDpadLeft());
        dpadRight = new DebouncedButton(driversGamepad.getDpadRight());


        aGreenButton = new DebouncedButton(driversGamepad.getAButton());
        bRedButton = new DebouncedButton(driversGamepad.getBButton());
        xBlueButton = driversGamepad.getXButton();
        yYellowButton = new DebouncedButton(driversGamepad.getYButton());
        leftBumper = driversGamepad.getLeftBumper();
        rightBumper = driversGamepad.getRightBumper();
        leftTrigger = driversGamepad.getLeftTrigger();
        rightTrigger = driversGamepad.getRightTrigger();

        // FIXME: Not yet mocked for tests
        lockButton = new DebouncedButton(driversGamepad.getLeftStickButton());
        unlockButton = new DebouncedButton(driversGamepad.getRightStickButton());
    }

    private void setupDerivedControls() {
        driveFastButton = new RangeInputButton(leftTrigger, 0.65f);
        driveInvertedButton = new RangeInputButton(rightTrigger, 0.65f);
        strafePriorityButton = driversGamepad.getYButton();
        fineStrafeLeft = driversGamepad.getDpadLeft();
        fineStrafeRight = driversGamepad.getDpadRight();
    }

    private boolean gripUpFirstTime = false;

    @Override
    public void periodicTask() {
        double x = driveStrafe.getPosition() * 1.1;
        double y = - driveForwardReverse.getPosition();
        double rot = driveRotate.getPosition(); // positive robot z rotation (human-normal) is negative joystick x axis
        boolean useEncoders = false;

        double topEndFactor = 1.6;

        if (liftMechanism != null && liftMechanism.isLiftAboveSlowHeight()) {
            topEndFactor = 2;
        }

        // do this first, it will be cancelled out by bump-strafe
        if (!(driveFastButton.isPressed())) {
            y /= topEndFactor;
            x /= 1.25;
            rot /= 1.3;
            useEncoders = false;
        }

        final boolean driveInverted;

        if (driveInvertedButton.isPressed()) {
            driveInverted = true;
        } else {
            driveInverted = false;
        }

        if (strafePriorityButton.isPressed()) {
            ((Drivebase)kinematics).useEncoders();
            y = 0;
            rot = 0;
        } else if (fineStrafeLeft.isPressed()) {
            ((Drivebase)kinematics).useEncoders();
            x = -.1;
        } else if (fineStrafeRight.isPressed()) {
            ((Drivebase)kinematics).useEncoders();
            x = .1;
        } else {
            ((Drivebase) kinematics).dontUseEncoders();
        }

        double xScaled = x;
        double yScaled = y;
        double rotateScaled = rot;

        kinematics.driveCartesian(xScaled, yScaled, rotateScaled, driveInverted);
    }
}
