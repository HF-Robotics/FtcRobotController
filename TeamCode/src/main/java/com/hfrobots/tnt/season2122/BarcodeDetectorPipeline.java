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

import com.hfrobots.tnt.season2021.GripPipelineHulls;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import lombok.Builder;
import lombok.Setter;

@Builder
public class BarcodeDetectorPipeline extends OpenCvPipeline {

    private static final String BARCODE_DETECTOR_TEL_CAPTION = "SDet";
    private final Telemetry telemetry;

    private final GripPipelineHulls gripPipeline = new GripPipelineHulls();

    private final Mat displayMat = new Mat();

    @Setter
    private volatile boolean startLookingForBarcode;

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(displayMat);

        // We will have two pipelines, one for detecting the tape markers (red or blue, depending
        // on alliance color), and another for detecting the rubber duck or team scoring element
        //
        // The first step is to run those pipelines. Each pipeline should return a set of contours
        // that will be scored.

        // The general flow for each pipeline is (from the Limelight docs for FRC):
        //
        // Most of this we will setup in GRiP, because it's easier to experiment there!
        //
        // * Threshold (usually via HSV)
        //
        //       * H is "hue", describes a pure color - making this range as small as possible
        //         will make this more accurate
        //
        //       * S is "saturation", a low value means the color is almost white, a high value
        //         means the color is completely pure (won't often happen in real life)
        //
        //       * V is "value", describes how much black is in the color, 0 means completely black.
        //         This value will need to be adjusted to avoid dark pixels from being considered
        //         by the pipeline.
        //
        // * Erode (if many objects are passing through HSV) / Dilate (used to patch holes)
        //
        // The pipeline should return contours, the rest of the code for sorting/scoring contours
        // lives in this class.


        //
        // To sort/score, there are a few options. We can use OpenCV to find the following from
        // the contours:
        //

        //
        // * Convex Hulls
        //
        // Imgproc.convexHull(MatOfPoint points, MatOfInt hull [,clockwise]);

        // * Bounding Rectangles
        //
        // Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
        // boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));

        // * Approximate centers:
        //
        // Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        //
        // or with rotated rectangles (which have centers defined) and a size (to compute area)
        //
        // Imgproc.minAreaRect(...)


        // Contour Filtering
        //
        // The goal is to ignore any contours which are smaller or larger than what our vision target
        // looks like when viewed from expected positions.
        //
        // We can filter these based on area, either on bounding rectangle or computed area of
        // the contour.
        //
        // We can use geometric properties (aspect ratio, vertical-ness, horizontal-ness, etc) to
        // filter contours to be the actual targets, vs something else (shoes, lights, etc).
        //
        // (calculate aspect ratio from the bounding rectangle width and height)
        //
        // We can sometimes exclude contours if they are outside the expected position in the
        // camera view. Most FTC vision detection during autonomous has fixed positions of
        // the item(s) being detected, as well as a fixed position for the robot, which means
        // there is only a certain area of the view that will ever contain detectable objects.
        //

        //
        // Once we have the "best" contours it helps to display them; for debugging during
        // development, as well as indicate correct setup before match play, or in the pits.

        // Imgproc.drawContours(displayMat, contours, -1, new Scalar(255, 30, 30), 2);

        if (startLookingForBarcode) {
            // Actually do the logic for finding the barcode position if requested.
            //
            // We run the pipeline during init(), to make sure it's working,
            // but we don't look for the randomized game piece until after auto starts
        }

        // This makes sure we get output at the driver station - which will include the helpful
        // visual information we've added.

        return displayMat;
    }
}
