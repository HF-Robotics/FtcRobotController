package com.hfrobots.tnt.season2122;

import com.hfrobots.tnt.corelib.vision.OpenCvTestTeleop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvPipeline;

@SuppressWarnings("unused")
@TeleOp(name="00 Barcode Tester", group="Utilities")
@Disabled
public class BarcodePipelineTester extends OpenCvTestTeleop {
    @Override
    protected OpenCvPipeline getPipeline() {
        return new BarcodeDetectorPipeline(telemetry, true);
    }
}
