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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.hfrobots.tnt.corelib.drive.mecanum.DriveConstants;

@Config
public class FreightFrenzyDriveConstants extends DriveConstants {

    private static DriveConstraints DRIVE_CONSTRAINTS = new DriveConstraints(80, 30.0, 0.0,
            Math.toRadians(27), Math.toRadians(90.0), 0.0);
    private static PIDCoefficients TRANSLATIONAL_PID_COEFFICIENTS = new PIDCoefficients(4.2D, 0, 0);
    private static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0.295D, 0, 0);

    @Override
    public DriveConstraints getBaseConstraints() {
        return DRIVE_CONSTRAINTS;
    }

    @Override
    public double getTrackWidth() {
        return 11;
    }

    @Override
    public PIDCoefficients getTranslationalPID() {
        return TRANSLATIONAL_PID_COEFFICIENTS;
    }

    @Override
    public PIDCoefficients getHeadingPid() {
        return HEADING_PID_COEFFICIENTS;
    }

    @Override
    public DriveConstraints getDriveConstraints() {
        return DRIVE_CONSTRAINTS;
    }

    @Override
    public double getLateralMultiplier() {
        return 2.18;
    }
}
