/*
 Copyright (c) 2026 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.util.templates;

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

// FIXME: Copy this class into the season package, and rename it to include the season name
// to avoid referencing controls of prior seasons, e.e. CenterstageDriverControls,
// PowerplayDriverControls, etc.
public class TemplateDriverControls implements PeriodicTask {
    protected InitLoopConfigTask autoConfigTask;

    protected RangeInput leftStickX;

    protected RangeInput leftStickY;

    protected RangeInput rightStickX;

    protected RangeInput rightStickY;

    protected OnOffButton dpadUp;

    protected OnOffButton dpadDown;

    protected OnOffButton dpadLeft;

    protected OnOffButton dpadRight;

    protected OnOffButton xBlueButton;

    protected OnOffButton bRedButton;

    protected OnOffButton yYellowButton;

    protected OnOffButton aGreenButton;

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

    private NinjaGamePad driversGamepad;

    private OpenLoopMecanumKinematics kinematics;

    private final float throttleGain = 0.4F;

    private final float throttleExponent = 5; // MUST BE AN ODD NUMBER!

    private final float throttleDeadband = 0;

    private final float lowPassFilterFactor = 1.0F;

    private DebouncedButton leftBumperDebounced;

    private DebouncedButton rightBumperDebounced;

    private DebouncedButton bRedButtonDebounced;

    private DebouncedButton xBlueButtonDebounced;

    private DebouncedButton dpadUpDebounced;

    private DebouncedButton dpadDownDebounced;

    @Builder
    private TemplateDriverControls(RangeInput leftStickX,
                                   RangeInput leftStickY,
                                   RangeInput rightStickX,
                                   RangeInput rightStickY,
                                   OnOffButton dpadUp,
                                   OnOffButton dpadDown,
                                   OnOffButton dpadLeft,
                                   OnOffButton dpadRight,
                                   OnOffButton xBlueButton,
                                   OnOffButton bRedButton,
                                   OnOffButton yYellowButton,
                                   OnOffButton aGreenButton,
                                   OnOffButton rightBumper,
                                   OnOffButton leftBumper,
                                   RangeInput leftTrigger,
                                   RangeInput rightTrigger,
                                   NinjaGamePad driversGamepad,
                                   OpenLoopMecanumKinematics kinematics,
                                   InitLoopConfigTask autoConfigTask)  {
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

        this.kinematics = kinematics;
        this.autoConfigTask = autoConfigTask;
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

        dpadDown = driversGamepad.getDpadDown();
        dpadUp = driversGamepad.getDpadUp();
        dpadLeft = driversGamepad.getDpadLeft();
        dpadRight = driversGamepad.getDpadRight();

        aGreenButton = driversGamepad.getAButton();
        bRedButton = driversGamepad.getBButton();
        xBlueButton = driversGamepad.getXButton();
        yYellowButton = driversGamepad.getYButton();

        leftBumper = driversGamepad.getLeftBumper();
        rightBumper = driversGamepad.getRightBumper();

        leftTrigger = driversGamepad.getLeftTrigger();
        rightTrigger = driversGamepad.getRightTrigger();

        // FIXME: Not yet mocked for tests
        lockButton = driversGamepad.getLeftStickButton().debounced();
        unlockButton = driversGamepad.getRightStickButton().debounced();
    }

    private void setupDerivedControls() {
        setupButtonsForAutoConfiguration();

        driveFastButton = new RangeInputButton(leftTrigger, 0.65f);
        driveInvertedButton = new RangeInputButton(rightTrigger, 0.65f);
    }

    private void setupButtonsForAutoConfiguration() {
        bRedButtonDebounced = bRedButton.debounced();
        xBlueButtonDebounced = xBlueButton.debounced();
        dpadUpDebounced = dpadUp.debounced();
        dpadDownDebounced = dpadDown.debounced();
        leftBumperDebounced = leftBumper.debounced();
        rightBumperDebounced = rightBumper.debounced();
    }

    @Override
    public void periodicTask() {
        if (autoConfigTask != null) {
            handleAutoConfig();
        } else {
            double x = driveStrafe.getPosition() * 1.1;
            double y = -driveForwardReverse.getPosition();
            double rot = driveRotate.getPosition(); // positive robot z rotation (human-normal) is negative joystick x axis

            if (!(driveFastButton.isPressed())) {
                y /= 1.25;
                x /= 1.25;
                rot /= 1.25;
            }

            final boolean driveInverted;

            if (driveInvertedButton.isPressed()) {
                driveInverted = true;
            } else {
                driveInverted = false;
            }

            double xScaled = x;
            double yScaled = y;
            double rotateScaled = rot;

            kinematics.driveCartesian(xScaled, yScaled, rotateScaled, driveInverted);
        }
    }

    private void handleAutoConfig() {
        if (lockButton.getRise()) {
            autoConfigTask.lockConfig();
        }

        if (unlockButton.getRise()) {
            autoConfigTask.unlockConfig();
        }

        // Use driver dpad up/down to select which route to run
        if (dpadDownDebounced.getRise()) {
            autoConfigTask.previousTaskChoice();
        } else if (dpadUpDebounced.getRise()) {
            autoConfigTask.nextTaskChoice();
        }

        // use left/right bumper to decrease/increase delay

        if (leftBumperDebounced.getRise()) {
            autoConfigTask.decreaseDelay();
        } else if (rightBumperDebounced.getRise()) {
            autoConfigTask.increaseDelay();
        }

        if (bRedButtonDebounced.getRise()) {
            autoConfigTask.chooseRedAlliance();
        } else if (xBlueButtonDebounced.getRise()) {
            autoConfigTask.chooseBlueAlliance();
        }
    }

    interface InitLoopConfigTask {
        void chooseBlueAlliance();

        void chooseRedAlliance();

        void nextTaskChoice();

        void previousTaskChoice();

        void increaseDelay();

        void decreaseDelay();

        void lockConfig();

        void unlockConfig();
    }
}
