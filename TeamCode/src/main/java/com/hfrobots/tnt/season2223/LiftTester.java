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

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.drive.NinjaMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.List;

@TeleOp(name = "Lift Tester")
@Disabled
public class LiftTester extends OpMode {
    
    private List<LynxModule> allHubs;

    private LiftMechanism liftMechanism;

    private OperatorControls operatorControls;
    
    @Override
    public void init() {
        final Ticker ticker = Ticker.systemTicker();

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        DigitalChannel lowerLimit = hardwareMap.get(DigitalChannel.class, "lowLimitSwitch");
        DigitalChannel higherLimit = hardwareMap.get(DigitalChannel.class, "highLimitSwitch");

        // Test the chain lift
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "chainMotor");
        
        liftMechanism = LiftMechanism.builder().liftMotor(NinjaMotor.asNeverest20(liftMotor))
                .lowerLiftLimit(lowerLimit)
                .upperLiftLimit(higherLimit)
                .telemetry(telemetry).build();

        NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

        operatorControls = OperatorControls.builder()
                .operatorGamepad(driversGamepad)
                .liftMechanism(liftMechanism).build();
    }
    
    @Override
    public void init_loop() {
        clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!
    }

    private void clearHubsBulkCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

        operatorControls.periodicTask();

        telemetry.update();
    }
}
