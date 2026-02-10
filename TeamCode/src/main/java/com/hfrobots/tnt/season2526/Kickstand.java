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

package com.hfrobots.tnt.season2526;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.google.common.base.Stopwatch;
import com.hfrobots.tnt.season2122.DriveTeamSignal;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

public class Kickstand {
    private static final int KICKSTAND_ENCODER_LIMIT = 1080;
    public static final double EXTENSION_POWER = .8;
    private final DcMotorEx kickstandMotor;

    private final DigitalChannel kickstandLimitswitch;


    private final static int RETRACTED_ENCODER_COUNT = 0; // we leave this at 0, it's where the motor started

    private final static Stopwatch extensionTimer = Stopwatch.createUnstarted();

    private final int encoderCountAtStart;

    private boolean limitSwitchTripped = false;

    private final DecodeDriveTeamSignal driveTeamSignal;

    public Kickstand(final HardwareMap hardwareMap, DecodeDriveTeamSignal driveTeamSignal) {
        kickstandMotor = hardwareMap.get(DcMotorEx.class, "kickstandMotor");
        kickstandLimitswitch = hardwareMap.get(DigitalChannel.class, "kickstandLimitswitch");
        this.driveTeamSignal = driveTeamSignal;
        encoderCountAtStart = kickstandMotor.getCurrentPosition();
    }

    public void extend() {
        if (limitSwitchTripped) {
            Log.i(LOG_TAG, "Limit switch tripped, not extending");
            stopExtending();
            return;
        }

        if (kickstandLimitswitch.getState() == false) {
            Log.i(LOG_TAG, "Reached limit switch, stop moving kickstand");
            limitSwitchTripped = true;
            stopExtending();

            if (driveTeamSignal != null) {
                driveTeamSignal.setKickstandFullyDeployed(true);
            }

            return;
        }

//        if (encoderCountIsNotAdvancing()) {
//            Log.i(LOG_TAG, "Encoder count stalled, stop moving kickstand");
//            stopExtending();
//
//            return;
//        }

        if (atOrBeyondExtensionLimit()) {
            Log.i(LOG_TAG, "Motor at extension limit, stop moving kickstand");
            stopExtending();

            if (driveTeamSignal != null) {
                driveTeamSignal.setKickstandFullyDeployed(true);
            }

            return;
        }

        if (atOrBeyondTimeLimit()) {
            Log.i(LOG_TAG, "Extension ran for too long, stop moving kickstand");
            stopExtending();

            return;
        }

        if (!extensionTimer.isRunning()) {
            extensionTimer.start();
        }

        kickstandMotor.setPower(EXTENSION_POWER);
    }

    private boolean encoderCountIsNotAdvancing() {
        long elapsed = extensionTimer.elapsed(TimeUnit.MILLISECONDS);

        if (elapsed > 250) {
            if (Math.abs(kickstandMotor.getCurrentPosition() - encoderCountAtStart) < 100) {
                return true;
            }
        }

        return false;
    }

    private boolean atOrBeyondTimeLimit() {
        // FIXME: actually calculate this

        long elapsedExtensionTimeMs = extensionTimer.elapsed(TimeUnit.MILLISECONDS);

        return false;
    }

    private boolean atOrBeyondExtensionLimit() {
        int currentPosition = kickstandMotor.getCurrentPosition();
        int delta = currentPosition - encoderCountAtStart;

        return delta >= KICKSTAND_ENCODER_LIMIT;
    }

    public void stopExtending() {
        kickstandMotor.setPower(0);
        if (extensionTimer.isRunning()) {
            extensionTimer.stop();
        }
    }
}
