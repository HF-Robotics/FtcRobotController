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

package com.hfrobots.tnt.season2526;

import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

import lombok.Setter;

public class DecodeDriveTeamSignal implements PeriodicTask {
    public static final double BLUE_LED = 0.611;

    public static final double RED_LED = 0.282;

    public static final double GREEN_LED = 0.5;

    public static final double AZURE_LED = 0.555;

    public static final double ORANGE_LED = 0.333;

    public static final double VIOLET_LED = .722;

    private final Servo ledIndicatorFrontRight;

    private final Servo ledIndicatorFrontLeft;

    private final Servo ledIndicatorBack;
    private final static int MATCH_DURATION_SECONDS = 120;

    private final static int END_GAME_SECONDS = MATCH_DURATION_SECONDS - 20;

    private final static int GO_TO_END_GAME_SECONDS = END_GAME_SECONDS - 10;

    private final Stopwatch stopwatch;

    private final Gamepad driverGamepad;

    private final Gamepad operatorGamepad;

    private static Constants.Alliance chosenAlliance = null;
    private boolean haveSentReadyForEndGameRumble;
    private boolean haveEndGameRumble;

    @Setter
    private boolean forceEndGameState = false;

    public enum DetectedIntakeArtifact {NONE, PURPLE, GREEN}

    @Setter
    DetectedIntakeArtifact detectedIntakeArtifact = DetectedIntakeArtifact.NONE;

    public DecodeDriveTeamSignal(final HardwareMap hardwareMap, final Ticker ticker,
                                 final Gamepad driverGamepad,
                                 final Gamepad operatorGamepad) {

        stopwatch = Stopwatch.createUnstarted(ticker);
        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGamepad;

        ledIndicatorBack = hardwareMap.get(Servo.class, "ledIndicatorBack");
        ledIndicatorFrontLeft = hardwareMap.get(Servo.class, "ledIndicatorFrontLeft");
        ledIndicatorFrontRight = hardwareMap.get(Servo.class, "ledIndicatorFrontRight");
    }

    public void startMatch() {
        stopwatch.start();
    }
    
    @Override
    public void periodicTask() {
        // FIXME: Called every time the OpMode loop()s - do things here that need to happen
        //        during the match
        //
        // There are probably a set of conditions that don't depend on anything but
        // elapsed time (start, almost end game, end game, almost end of match) that
        // we need to calculate and detect.
        //
        // There are also probably later external events that we'd like to indicate,
        // for example, "unsafe" is being pressed, or the lift is at a certain level,
        // etc.
        //
        // On top of all of that, we need to determine a priority for displaying the signal
        // which condition, should take priority, since we only have one LED light strip
        // for now, it can only show one thing at a time! Which one is most important
        // is up to you, the drive team, to decide.

        if (isEndGame()) {
            if (!haveEndGameRumble) {
                driverGamepad.rumble(750);
                operatorGamepad.rumble(750);
                haveEndGameRumble = true;
            }
            setIndicators(GREEN_LED);
        } else if (readyForEndGame()) {
            if (!haveSentReadyForEndGameRumble) {
                driverGamepad.rumbleBlips(3);
                operatorGamepad.rumbleBlips(3);
                haveSentReadyForEndGameRumble = true;
            }
            setIndicators(ORANGE_LED);
        } else {
            if (chosenAlliance != null) {
                switch (chosenAlliance) {
                    case RED: {
                        setIndicators(RED_LED);
                        break;
                    }
                    case BLUE: {
                        setIndicators(BLUE_LED);
                        break;
                    }
                    default:
                        setIndicators(0);
                        break;
                }
            } else {
                setIndicators(VIOLET_LED);
            }
        }
    }

    private void setIndicators(double value) {
        //ledIndicatorBack.setPosition(value);
        ledIndicatorFrontLeft.setPosition(value);
        ledIndicatorFrontRight.setPosition(value);
    }

    public boolean matchStarted() {
        return stopwatch.isRunning();
    }

    public boolean readyForEndGame() {
        return hasEnoughTimePassed(GO_TO_END_GAME_SECONDS);
    }

    private boolean hasEnoughTimePassed(int seconds) {
        long elapsedTimeSec = stopwatch.elapsed(TimeUnit.SECONDS);

        if (elapsedTimeSec >= seconds) {
            return true;
        } else {
            return false;
        }
    }

    public void setAlliance(final Constants.Alliance alliance) {
        chosenAlliance = alliance;
    }

    public boolean isEndGame() {
        if (forceEndGameState) {
            return true;
        }

        return hasEnoughTimePassed(END_GAME_SECONDS);
    }
}
