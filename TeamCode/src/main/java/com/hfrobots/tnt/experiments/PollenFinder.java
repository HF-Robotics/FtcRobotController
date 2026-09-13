package com.hfrobots.tnt.experiments;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PollenFinder implements VisionProcessor {
    // Pre-allocate memory matrices to prevent memory leaks
    private Mat hsvMat = new Mat();
    private Mat maskMat = new Mat();
    private Mat hierarchyMat = new Mat();
    private Mat maskedOutputMat = new Mat();

    // Define HSV thresholds (Example: Orange Rings)
    public static Scalar LOWER_BOUND = new Scalar(14.2, 65.8, 107.7);
    public static Scalar UPPER_BOUND = new Scalar(255, 255, 213.9);
    public static double MIN_CONTOUR_AREA = 500.0;

    // Public variables to read target data from your OpMode
    public boolean targetFound = false;
    public int centerX = 0;
    public int centerY = 0;

    // EOCV-Sim will automatically detect and inject its telemetry instance here
    private Telemetry telemetry;

    public PollenFinder(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Executed once when the processor is attached to the VisionPortal
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // 1. Convert the full-color frame to HSV color space
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // 2. Isolate the target color (creates a Black & White mask)
        Core.inRange(hsvMat, LOWER_BOUND, UPPER_BOUND, maskMat);

        // 3. Find contours using the binary mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
            maskMat, 
            contours, 
            hierarchyMat, 
            Imgproc.RETR_EXTERNAL,       
            Imgproc.CHAIN_APPROX_SIMPLE  
        );

        // 4. Track down the largest contour matching our target size
        MatOfPoint largestContour = null;
        double maxArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA && area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        // 5. Mask the frame: Only matching pixels retain full color; everything else turns black
        maskedOutputMat.release(); 
        Core.bitwise_and(frame, frame, maskedOutputMat, maskMat);

        // 6. Draw the green bounding box on top of the full-color masked matrix
        if (largestContour != null) {
            Rect boundingBox = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(
                maskedOutputMat, 
                boundingBox, 
                new Scalar(0, 255, 0), // Green bounding box
                2
            );

            // Update telemetry coordinates safely
            targetFound = true;
            centerX = boundingBox.x + (boundingBox.width / 2);
            centerY = boundingBox.y + (boundingBox.height / 2);
        } else {
            targetFound = false;
        }

        if (targetFound) {
            telemetry.addData("[Processor] Target Found", "Yes");
            telemetry.addData("[Processor] Center X", centerX);
            telemetry.addData("[Processor] Center Y", centerY);
            telemetry.addData("[Processor] Area", maxArea);
        } else {
            telemetry.addData("[Processor] Target Found", "No");
        }
        
        // EOCV-Sim handles the update loop, but calling update() ensures immediate rendering
        telemetry.update(); 
        
        // 7. Copy our custom visualization matrix BACK into the frame matrix.
        // VisionPortal requires us to modify the 'frame' variable to change what is displayed.
        maskedOutputMat.copyTo(frame);

        // Return null since we don't need to pass extra user data to onDrawFrame
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasPxToBmpPx, Object userContext) {
        // Android Canvas drawing method. We leave this empty because we 
        // drew our visual feedback directly onto the OpenCV Mat in processFrame.
    }
}
