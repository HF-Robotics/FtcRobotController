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

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.metrics.RobotMetricsSampler;
import com.ftc9929.metrics.StatsdMetricsReporter;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.hfrobots.tnt.season2324.Shared;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = DecodeDriverControlled.OP_MODE_NAME)
public class DecodeDriverControlled extends OpMode {
    public static final String OP_MODE_NAME = "00 DECODE";

    private final boolean emitMetrics = false;

    private DecodeDrivebase drivebase;

    private DecodeDriverControls driverControls;

    private DecodeOperatorControls operatorControls;

    private StatsDMetricSampler legacyMetricsSampler;

    private RobotMetricsSampler newMetricsSampler;;

    private final boolean useLegacyMetricsSampler = true;

    private List<LynxModule> allHubs;

    private WheeledLauncher launcher;

    private Carousel carousel;

    private DecodeDriveTeamSignal driveTeamSignal;
    private WebcamManualControlSetup manualControlSetup;

    @Override
    public void init() {
        Shared.withBetterErrorHandling(() -> {
            final Ticker ticker = Ticker.systemTicker();

            drivebase = new DecodeDrivebase(hardwareMap);

            try {
                launcher = new WheeledLauncher(hardwareMap);
            } catch (IllegalArgumentException ex) {
                launcher = null;
            }

            NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

            driverControls = DecodeDriverControls.builder()
                    .driversGamepad(driversGamepad)
                    .launcher(launcher)
                    .kinematics(drivebase).build();

            RollerIntake intake;

            try {
                intake = new RollerIntake(hardwareMap);
            } catch (IllegalArgumentException ex) {
                intake = null;
            }

            try {
                carousel = new Carousel(hardwareMap);
            } catch (IllegalArgumentException ex) {
                carousel = null;
            }

            final NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

            operatorControls = DecodeOperatorControls.builder()
                    .operatorGamepad(operatorGamepad)
                    .intake(intake)
                    .carousel(carousel)
                    .launcher(launcher).build();

            setupMetricsSampler(driversGamepad, operatorGamepad);

            try {
                driveTeamSignal = new DecodeDriveTeamSignal(hardwareMap, ticker, gamepad1, gamepad2);
            } catch (IllegalArgumentException ex) {
                driveTeamSignal = null;
            }

            try {
                initAprilTag(ticker);
                canUseAprilTags = true;
            } catch (Exception ex) {
                Log.e(LOG_TAG, "Unable to initialize AprilTag", ex);
                canUseAprilTags = false;
            }

            allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        });
    }

    private void setupMetricsSampler(NinjaGamePad driversGamepad, NinjaGamePad operatorGamepad) {
        if (emitMetrics) {
            try {
                if (useLegacyMetricsSampler) {
                    legacyMetricsSampler = new StatsDMetricSampler(hardwareMap, driversGamepad, operatorGamepad);
                } else {
                    StatsdMetricsReporter metricsReporter = StatsdMetricsReporter.builder()
                            .metricsServerHost("192.168.43.78").
                            metricsServerPortNumber(8126).build();

                    newMetricsSampler = RobotMetricsSampler.builder()
                            .metricsReporter(metricsReporter)
                            .hardwareMap(hardwareMap)
                            .driverControls(driversGamepad)
                            .operatorControls(operatorGamepad).build();
                }
            } catch (Exception ex) {
                Log.w(LOG_TAG, "Unable to setup metrics sampler", ex);
            }
        }
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
        Shared.withBetterErrorHandling(() -> {
            super.start();

            if (driveTeamSignal != null) {
                driveTeamSignal.startMatch();
            }
        });
    }

    @Override
    public void loop() {
        Shared.withBetterErrorHandling(() -> {
            clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

            if (canUseAprilTags) {
                manualControlSetup.periodicTask();
            }
            
            if (manualControlSetup.isCameraIsSetup() && driverControls.rightBumper.isPressed()) {
                doAimWithAprilTagsStuff();
            } else {
                driverControls.periodicTask();
            }
            
            operatorControls.periodicTask();

            if (driveTeamSignal != null) {
                driveTeamSignal.periodicTask();
            }

            if (emitMetrics) {
                if (useLegacyMetricsSampler) {
                    if (legacyMetricsSampler != null) {
                        legacyMetricsSampler.doSamples();
                    }
                } else {
                    if (newMetricsSampler != null) {
                        newMetricsSampler.doSamples();
                    }
                }
            }

            if (launcher != null) {
                launcher.updateTelemetry(telemetry);
            }

            telemetry.update();
        });
    }

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 60.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private void doAimWithAprilTagsStuff() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.

                if (detection.id == 24 || detection.id == 20) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            drivebase.driveCartesian(-strafe, drive, -turn, false);
        }
    }

    // April Tag Stuff
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(final Ticker ticker) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
            .addProcessor(aprilTag)
            .build();

        manualControlSetup = new WebcamManualControlSetup(
                visionPortal, telemetry, ticker);
    }

    boolean canUseAprilTags = false;
}
