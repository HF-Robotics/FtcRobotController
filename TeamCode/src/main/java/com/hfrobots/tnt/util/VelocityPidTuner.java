/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.util;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name="Velocity PID", group="Utilities")
@Configurable
public class VelocityPidTuner extends OpMode {
    private List<NamedDeviceMap.NamedDevice<DcMotorEx>> namedMotors;
    private Map<DcMotorEx, String> motorsToNames = new HashMap<>();
    private int currentListPosition;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static PanelsTelemetry telemetryP = PanelsTelemetry.INSTANCE;

    private DebouncedButton aButton;

    private DebouncedButton bButton;

    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

    private DebouncedButton dpadLeft;

    private DebouncedButton dpadRight;

    private static final double FAR_LAUNCH_VELOCITY = 1598;

    private static final double MEDIUM_LAUNCH_VELOCITY = 1300;

    private static final double CLOSE_LAUNCH_VELOCITY = 1080;

    public static double kP;

    public static double kI;

    public static double kD;

    public static double kF;

    private double requestedLaunchVelocity = 0;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        NamedDeviceMap namedDeviceMap = new NamedDeviceMap(hardwareMap);
        namedMotors = namedDeviceMap.getAll(DcMotorEx.class);
        currentListPosition = 0;

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        aButton = ninjaGamePad.getAButton().debounced();
        bButton = ninjaGamePad.getBButton().debounced();
        rightBumper = ninjaGamePad.getRightBumper().debounced();
        dpadUp = ninjaGamePad.getDpadUp().debounced();
        dpadDown = ninjaGamePad.getDpadDown().debounced();
        dpadLeft = ninjaGamePad.getDpadLeft().debounced();
        dpadRight = ninjaGamePad.getDpadRight().debounced();

        PIDFCoefficients currentPIDF = hardwareMap.get(DcMotorEx.class, "launcherMotor").getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        kP = currentPIDF.p;
        kI = currentPIDF.i;
        kD = currentPIDF.d;
        kF = currentPIDF.f;
    }

    @Override
    public void loop() {

        if (namedMotors.isEmpty()) {
            telemetry.addData("No DC Motors", "");
            updateTelemetry(telemetry);
            return;
        }

        /*
        if (rightBumper.getRise()) {
            namedMotors.get(currentListPosition).getDevice().setVelocity(0);

            currentListPosition++;

            if (currentListPosition == namedMotors.size()) {
                currentListPosition = 0;
            }

            PIDFCoefficients currentPIDF = namedMotors.get(currentListPosition).getDevice().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            kP = currentPIDF.p;
            kI = currentPIDF.i;
            kD = currentPIDF.d;
            kF = currentPIDF.f;
        }

        NamedDeviceMap.NamedDevice<DcMotorEx> namedDcMotor = namedMotors.get(currentListPosition);
        */
        DcMotorEx currentMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");

        String motorName = "launcher"; // namedDcMotor.getName();

        if (dpadUp.getRise()) {
            requestedLaunchVelocity = FAR_LAUNCH_VELOCITY;
        } else if (dpadLeft.getRise()) {
            requestedLaunchVelocity = MEDIUM_LAUNCH_VELOCITY;
        } else if (dpadDown.getRise()) {
            requestedLaunchVelocity = CLOSE_LAUNCH_VELOCITY;
        } else if (dpadRight.getRise()) {
            requestedLaunchVelocity = 0;
        }

        if (aButton.getRise()) {
            gamepad1.rumbleBlips(1);
            currentMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }

        currentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentMotor.setVelocity(requestedLaunchVelocity);

        double encoderClicksPerSec = currentMotor.getVelocity();

        telemetry.addData("motor ",  "%s - vel_req %s - cur_vel %s",
                motorName,
                Double.toString(requestedLaunchVelocity),
                Double.toString(encoderClicksPerSec));
        PIDFCoefficients currentPidF = currentMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PIDF", currentPidF.p + " " + currentPidF.i + " " + currentPidF.d + " " + currentPidF.f);
        //updateTelemetry(telemetry);


        telemetryM.addData("reqVel", requestedLaunchVelocity);
        telemetryM.addData("curVel", encoderClicksPerSec);
        telemetryM.update(telemetry);
    }
}
