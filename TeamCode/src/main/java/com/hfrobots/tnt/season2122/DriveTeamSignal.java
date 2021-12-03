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

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;

import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

public class DriveTeamSignal implements PeriodicTask {
    private final RevBlinkinLedDriver blinkinLed;

    private final Stopwatch stopwatch;

    public DriveTeamSignal(final HardwareMap hardwareMap, final Ticker ticker) {
        blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        stopwatch = Stopwatch.createUnstarted(ticker);
    }

    public void startMatch() {
        // FIXME: Start doing whatever we need to do when the match starts
    }

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

        blinkinLed.setPattern(blinkinPattern);
    }

    public boolean matchStarted() {
        return false; // FIXME
    }

    public boolean readyForEndGame() {
        return false; // FIXME
    }

    public boolean isEndGame() {
        return false; // FIXME
    }

    public boolean startWarehouseRun() {
        return false; // FIXME
    }
}
