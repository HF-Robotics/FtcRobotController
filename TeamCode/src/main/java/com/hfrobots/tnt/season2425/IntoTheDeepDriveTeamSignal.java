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

package com.hfrobots.tnt.season2425;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;

import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

public class IntoTheDeepDriveTeamSignal implements PeriodicTask {
    private RevBlinkinLedDriver blinkinLed;

    private final static RevBlinkinLedDriver.BlinkinPattern GO_TO_END_GAME_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;

    private final static RevBlinkinLedDriver.BlinkinPattern END_GAME_PATTERN = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;

    private final static RevBlinkinLedDriver.BlinkinPattern RED_ALLIANCE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;

    private final static RevBlinkinLedDriver.BlinkinPattern BLUE_ALLIANCE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    private final static int MATCH_DURATION_SECONDS = 120;

    private final static int END_GAME_SECONDS = MATCH_DURATION_SECONDS - 30;

    private final static int GO_TO_END_GAME_SECONDS = END_GAME_SECONDS - 10;

    private final Stopwatch stopwatch;

    private final Gamepad driverGamepad;

    private final Gamepad operatorGamepad;

    private static Constants.Alliance chosenAlliance = null;
    private boolean haveSentReadyForEndGameRumble;
    private boolean haveEndGameRumble;

    public IntoTheDeepDriveTeamSignal(final HardwareMap hardwareMap, final Ticker ticker,
                                      final Gamepad driverGamepad,
                                      final Gamepad operatorGamepad) {
        try {
            blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        } catch (Throwable t) {
            blinkinLed = null;
        }

        stopwatch = Stopwatch.createUnstarted(ticker);
        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGamepad;
    }

    public void startMatch() {
        stopwatch.start();
    }

    private RevBlinkinLedDriver.BlinkinPattern currentBlinkinPattern = null;

    @Override
    public void periodicTask() {
        RevBlinkinLedDriver.BlinkinPattern blinkinPattern = BREATH_GRAY;

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
            blinkinPattern = END_GAME_PATTERN;
        } else if (readyForEndGame()) {
            if (!haveSentReadyForEndGameRumble) {
                driverGamepad.rumbleBlips(3);
                operatorGamepad.rumbleBlips(3);
                haveSentReadyForEndGameRumble = true;
            }
            blinkinPattern = GO_TO_END_GAME_PATTERN;
        } else if (chosenAlliance != null) {
           switch (chosenAlliance) {
               case RED: {
                   blinkinPattern = RED_ALLIANCE_PATTERN;
                   break;
               }
               case BLUE: {
                   blinkinPattern = BLUE_ALLIANCE_PATTERN;
                   break;
               }
           }
        }

        // Only set if changed - the driver logs every time setPattern() is called :(
        if (currentBlinkinPattern == null || blinkinPattern != currentBlinkinPattern) {
            if (blinkinLed != null) {
                blinkinLed.setPattern(blinkinPattern);
            }

            currentBlinkinPattern = blinkinPattern;
        }
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
        return hasEnoughTimePassed(END_GAME_SECONDS);
    }
}
