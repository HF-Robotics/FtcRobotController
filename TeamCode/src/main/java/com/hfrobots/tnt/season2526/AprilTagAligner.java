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

import com.bylazar.configurables.annotations.Configurable;
import com.google.common.base.Ticker;
import com.google.common.collect.ImmutableSet;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Set;


@Configurable
public class AprilTagAligner implements PeriodicTask {

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 60.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public static double TURN_GAIN   =  0.05  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private final int BLUE_TARGET_ID = 20;

    private final int RED_TARGET_ID = 24;

    private final ImmutableSet<Integer> BOTH_GOAL_TAGS = ImmutableSet.of(BLUE_TARGET_ID, RED_TARGET_ID);

    private final ImmutableSet<Integer> BLUE_GOAL_TAG = ImmutableSet.of(BLUE_TARGET_ID);

    private final ImmutableSet<Integer> RED_GOAL_TAG = ImmutableSet.of(RED_TARGET_ID);

    private final Telemetry telemetry;

    private final DecodeDrivebase drivebase;

    private final HardwareMap hardwareMap;

    private final Ticker ticker;

    private VisionPortal visionPortal;               // Used to manage the video source.

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    public AprilTagAligner(Telemetry telemetry, DecodeDrivebase drivebase, HardwareMap hardwareMap, Ticker ticker) {
        this.telemetry = telemetry;
        this.drivebase = drivebase;
        this.hardwareMap = hardwareMap;
        this.ticker = ticker;

        initAprilTag(ticker);
    }

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

    public void periodicCameraSetup() {
        manualControlSetup.periodicTask();
    }

    private WebcamManualControlSetup manualControlSetup;

    public Double getBearing(final Constants.Alliance alliance) {
        // Different than other times, we want to only use tag from
        // our alliance, since the robot can see *both*

        final ImmutableSet<Integer> correctTags;

        if (alliance == Constants.Alliance.BLUE) {
            correctTags = BLUE_GOAL_TAG;
        } else {
            correctTags = RED_GOAL_TAG;
        }

        desiredTag = detectTag(correctTags);

        // Find the tag, return the distance or *null* if the tag isn't found
        if (desiredTag != null) {
            Log.i(LOG_TAG, "Found tag " + desiredTag.id);

            return desiredTag.ftcPose.bearing;
        } else {
            return null;
        }
    }

    public Double getRange() {
        if (!manualControlSetup.isCameraIsSetup()) {
            telemetry.addData("Apriltags", "Camera isn't ready");
        }

        desiredTag = detectTag(BOTH_GOAL_TAGS);
        // Find the tag, return the distance or *null* if the tag isn't found
        if (desiredTag != null)
        {
            return desiredTag.ftcPose.range;
        }
        else
        {
            return null;
        }
    }

    public void aimToDetectedAprilTag(Constants.Alliance currentAlliance) {
        final ImmutableSet<Integer> lookForTagsFrom;

        if (currentAlliance == null) {
            lookForTagsFrom = BOTH_GOAL_TAGS;
        } else if (currentAlliance == Constants.Alliance.BLUE) {
            lookForTagsFrom = BLUE_GOAL_TAG;
        } else if (currentAlliance == Constants.Alliance.RED) {
            lookForTagsFrom = RED_GOAL_TAG;
        } else {
            lookForTagsFrom = BOTH_GOAL_TAGS;
        }

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        desiredTag = null;

        desiredTag = detectTag(lookForTagsFrom);
        targetFound = desiredTag != null;

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }

        if (targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // headingError = headingError + 2;
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            if (drivebase != null) {
                drivebase.driveCartesian(0 /* -strafe */, 0/* drive */, -turn, false);
            }

        }
    }

    private AprilTagDetection detectTag(final Set<Integer> desiredTags) {
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections != null) {
            Log.d(LOG_TAG, "Detections: " + currentDetections);
        }

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.

                if (desiredTags.contains(detection.id)) {
                    // Yes, we want to use this tag.
                    return detection;
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    return null;
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);

                return null;
            }
        }

        return null;
    }

    @Override
    public void periodicTask() {

    }
}
