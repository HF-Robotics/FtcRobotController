package com.hfrobots.tnt.season2526;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.hfrobots.tnt.fakes.FakeTelemetry;
import com.hfrobots.tnt.season2526.mechanisms.ArtifactDetector;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;

public class ArtifactDetectorTest {
    private final Servo colorIndicator;

    private final ArtifactDetector detector;

    private final RevColorSensorV3 artifactColor1;

    private final RevColorSensorV3 artifactColor2;

    private final FakeTelemetry telemetry = new FakeTelemetry();

    public ArtifactDetectorTest() {
        detector = new ArtifactDetector(DecodeTestConstants.HARDWARE_MAP, telemetry);
        artifactColor1 = DecodeTestConstants.HARDWARE_MAP.get(RevColorSensorV3.class, "artifactColor1");
        artifactColor2 = DecodeTestConstants.HARDWARE_MAP.get(RevColorSensorV3.class, "artifactColor2");
        colorIndicator = DecodeTestConstants.HARDWARE_MAP.get(Servo.class, "ledIndicatorBack");
    }

    @Test
    public void happyPath() {
        // When red and blue are > green, then we think we see purple
        //((FakeLynxI2cColorRangeSensor)artifactColor1).setRed(80);
        //((FakeLynxI2cColorRangeSensor)artifactColor1).setBlue(80);
        //((FakeLynxI2cColorRangeSensor)artifactColor1).setGreen(60);

        //((FakeLynxI2cColorRangeSensor)artifactColor2).setRed(80);
        //((FakeLynxI2cColorRangeSensor)artifactColor2).setBlue(80);
        //((FakeLynxI2cColorRangeSensor)artifactColor2).setGreen(60);

        // Force the artifact detector to do one sample
        //detector.periodicTask();

        //assertEquals(VIOLET_LED,  colorIndicator.getPosition(), 0.01);
    }
}
