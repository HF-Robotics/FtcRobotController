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

package com.hfrobots.tnt.season2526.driveteam;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.LowPassFilteredRangeInput;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.ParametricScaledRangeInput;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.ftc9929.corelib.drive.OpenLoopMecanumKinematics;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.hfrobots.tnt.season2526.mechanisms.Kickstand;
import com.hfrobots.tnt.season2526.mechanisms.WheeledLauncher;

import lombok.Builder;

// FIXME: Copy this class into the season package, and rename it to include the season name
// to avoid referencing controls of prior seasons, e.e. CenterstageDriverControls,
// PowerplayDriverControls, etc.
public class DecodeDriverControls implements PeriodicTask {
    protected InitLoopConfigTask autoConfigTask;

    protected RangeInput leftStickX;

    protected RangeInput leftStickY;

    protected RangeInput rightStickX;

    protected RangeInput rightStickY;

    protected DebouncedButton dpadUp;

    protected DebouncedButton dpadDown;

    protected DebouncedButton dpadLeft;

    protected DebouncedButton dpadRight;

    protected DebouncedButton xBlueButton;

    protected DebouncedButton bRedButton;

    protected OnOffButton yYellowButton;

    protected OnOffButton aGreenButton;

    private OnOffButton rightBumper;

    protected OnOffButton leftBumper;

    protected DebouncedButton rightBumperDebounced;

    protected DebouncedButton leftBumperDebounced;

    protected RangeInput leftTrigger;

    protected RangeInput rightTrigger;

    // Derived
    protected RangeInput driveForwardReverse;

    protected RangeInput driveStrafe;

    protected RangeInput driveRotate;

    protected DebouncedButton lockButton;

    protected DebouncedButton unlockButton;

    protected OnOffButton driveInvertedButton;

    protected OnOffButton driveSlowButton;

    private NinjaGamePad driversGamepad;

    private OpenLoopMecanumKinematics kinematics;

    private WheeledLauncher launcher;

    private final float throttleGain = 0.4F;

    private final float throttleExponent = 5; // MUST BE AN ODD NUMBER!

    private final float throttleDeadband = 0;

    private final float lowPassFilterFactor = 1.0F;

    private DebouncedButton launchSpeedFar;

    private DebouncedButton launchSpeedMed;

    private DebouncedButton launchSpeedClose;

    private DebouncedButton launchSpeedStop;

    private OnOffButton autoRange;

    private OnOffButton kickstandExtend;

    private DecodeDriveTeamSignal driveTeamSignal;

    private Kickstand kickstand;

    @Builder
    private DecodeDriverControls(RangeInput leftStickX,
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
                                 DebouncedButton xBlueButton,
                                 DebouncedButton bRedButton,
                                 OnOffButton yYellowButton,
                                 OnOffButton aGreenButton,
                                 OnOffButton rightBumper,
                                 OnOffButton leftBumper,
                                 RangeInput leftTrigger,
                                 RangeInput rightTrigger,
                                 NinjaGamePad driversGamepad,
                                 OpenLoopMecanumKinematics kinematics,
                                 InitLoopConfigTask autoConfigTask,
                                 WheeledLauncher launcher,
                                 DecodeDriveTeamSignal driveTeamSignal,
                                 Kickstand kickstand)  {
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
            this.leftBumperDebounced = this.leftBumper.debounced();
            this.rightBumperDebounced = this.getRightBumper().debounced();
        }

        setupDerivedControls();
        setupCurvesAndFilters();

        this.kinematics = kinematics;
        this.autoConfigTask = autoConfigTask;
        this.launcher = launcher;
        this.driveTeamSignal = driveTeamSignal;
        this.kickstand = kickstand;
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

        dpadDown = driversGamepad.getDpadDown().debounced();
        dpadUp = driversGamepad.getDpadUp().debounced();
        dpadLeft = driversGamepad.getDpadLeft().debounced();
        dpadRight = driversGamepad.getDpadRight().debounced();

        aGreenButton = driversGamepad.getAButton();
        bRedButton = driversGamepad.getBButton().debounced();
        xBlueButton = driversGamepad.getXButton().debounced();
        yYellowButton = driversGamepad.getYButton();

        leftBumper = driversGamepad.getLeftBumper();
        rightBumper = driversGamepad.getRightBumper();
        leftBumperDebounced = leftBumper.debounced();
        rightBumperDebounced = getRightBumper().debounced();

        leftTrigger = driversGamepad.getLeftTrigger();
        rightTrigger = driversGamepad.getRightTrigger();

        // FIXME: Not yet mocked for tests
        lockButton = driversGamepad.getLeftStickButton().debounced();
        unlockButton = driversGamepad.getRightStickButton().debounced();
    }

    private void setupDerivedControls() {
        driveSlowButton = new RangeInputButton(leftTrigger, 0.65f);
        driveInvertedButton = new RangeInputButton(rightTrigger, 0.65f);

        launchSpeedFar = dpadUp;

        launchSpeedMed = dpadLeft;

        launchSpeedClose = dpadDown;

        launchSpeedStop = bRedButton;

        autoRange = leftBumper;

        kickstandExtend = yYellowButton;
    }

    @Override
    public void periodicTask() {
        if (autoConfigTask != null) {
            handleAutoConfig();
        } else {
            double x = driveStrafe.getPosition() * 1.1;
            double y = -driveForwardReverse.getPosition();
            double rot = driveRotate.getPosition(); // positive robot z rotation (human-normal) is negative joystick x axis

            if (driveSlowButton.isPressed()) {
                y /= 2;
                x /= 2;
                rot /= 2;
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

        if (launcher != null) {
            if (launchSpeedFar.getRise()) {
                launcher.farLaunchVelocity();
            } else if (launchSpeedMed.getRise()) {
                launcher.mediumLaunchVelocity();
            } else if (launchSpeedClose.getRise()) {
                launcher.closeLaunchVelocity();
            } else if (launchSpeedStop.getRise()) {
                launcher.stopLauncher();
            } else if (autoRange.isPressed()) {
                launcher.autoRange();
            }
        }

        if (driveTeamSignal != null && kickstand != null) {
            if (driveTeamSignal.isEndGame() || (aGreenButton != null && aGreenButton.isPressed())) {
                if (kickstandExtend.isPressed()) {
                    kickstand.extend();
                } else {
                    kickstand.stopExtending();
                }
            }
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
        if (dpadDown.getRise()) {
            autoConfigTask.previousTaskChoice();
        } else if (dpadUp.getRise()) {
            autoConfigTask.nextTaskChoice();
        }

        // use left/right bumper to decrease/increase delay

        if (leftBumperDebounced.getRise()) {
            autoConfigTask.decreaseDelay();
        } else if (rightBumperDebounced.getRise()) {
            autoConfigTask.increaseDelay();
        }

        // Alliance selection
        if (bRedButton.getRise()) {
            autoConfigTask.chooseRedAlliance();
        } else if (xBlueButton.getRise()) {
            autoConfigTask.chooseBlueAlliance();
        }
    }

    public OnOffButton getRightBumper() {
        return rightBumper;
    }

    public interface InitLoopConfigTask {
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
