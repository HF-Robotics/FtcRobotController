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

import com.google.common.collect.Lists;
import com.hfrobots.tnt.season2122.pipelines.FindDuckContoursNaturalLighting;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

import lombok.Builder;
import lombok.Setter;

@Builder
public class BarcodeDetectorPipeline extends OpenCvPipeline {

    private static final String BARCODE_DETECTOR_TEL_CAPTION = "BcDet";

    private static final Scalar RGB_ORANGE = new Scalar(234, 126, 39);

    private static final Scalar RGB_PURPLE = new Scalar(152, 0, 62);

    private static final Scalar RGB_GREEN = new Scalar(0, 255, 0);

    private final Rect LEFT_ZONE = new Rect(0, 0, 106, 240);

    private final Rect CENTER_ZONE = new Rect(106, 0, 106, 240);

    private final Rect RIGHT_ZONE = new Rect(212, 0, 106, 240);

    private final Telemetry telemetry;

    private final FindDuckContoursNaturalLighting findDuckContours = new FindDuckContoursNaturalLighting();

    private final Mat displayMat = new Mat();

    @Setter
    private volatile boolean startLookingForBarcode;

    private final Mat rgbMat = new Mat();

    @Override
    public Mat processFrame(final Mat input) {
        input.copyTo(displayMat);

        // We will have two pipelines, one for detecting the tape markers (red or blue, depending
        // on alliance color), and another for detecting the rubber duck or team scoring element
        //
        // The first step is to run those pipelines. Each pipeline should return a set of contours
        // that will be scored.

        // OpenCV expects frames in BGR for things like HSV to work
        // EasyOpenCV sends in RGBA, must convert them before doing anything else
        Imgproc.cvtColor(input, rgbMat, Imgproc.COLOR_RGBA2BGR);

        // This is the pipeline, created in GRiP that isolates the duck in the image
        findDuckContours.process(rgbMat);

        // We'll look through the list of contours of detected ducks, but hopefully there's only one!
        List<MatOfPoint> filteredDuckContours = findDuckContours.filterContoursOutput();

        Imgproc.drawContours(displayMat, filteredDuckContours, -1, RGB_PURPLE, 2);

        // It's easier to deal with rectangles rather than the freeform shape of the duck
        List<Rect> duckBoundingRectangles = Lists.newArrayList();

        for (final MatOfPoint contour : filteredDuckContours) {
            final Rect boundingRect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(displayMat,  boundingRect.br(), boundingRect.tl(), RGB_GREEN, 4);

            duckBoundingRectangles.add(boundingRect);

            final Point duckCenterPoint = centerPoint(boundingRect);
            Imgproc.circle(displayMat, duckCenterPoint, 6, RGB_GREEN);
        }

        // Draw the bar code positions where ducks are expected
        Imgproc.rectangle(displayMat, LEFT_ZONE, RGB_ORANGE, 2);
        Imgproc.rectangle(displayMat, CENTER_ZONE, RGB_ORANGE, 2);
        Imgproc.rectangle(displayMat, RIGHT_ZONE, RGB_ORANGE, 2);

        if (startLookingForBarcode) {
            // Actually do the logic for finding the barcode position if requested.
            //
            // We run the pipeline during init(), to make sure it's working,
            // but we don't look for the randomized game piece until after auto starts

            if (duckBoundingRectangles.size() > 1) {
                // Uhhhhhh...FIXME - what do we here?
            } else if (duckBoundingRectangles.size() == 1) {
                final Rect duckBoundingRect = duckBoundingRectangles.get(0);

                final Point duckCenterPoint = centerPoint(duckBoundingRect);

                if (LEFT_ZONE.contains(duckCenterPoint)) {
                    Imgproc.rectangle(displayMat, LEFT_ZONE, RGB_GREEN, 2);
                } else if (CENTER_ZONE.contains(duckCenterPoint)) {
                    Imgproc.rectangle(displayMat, CENTER_ZONE, RGB_GREEN, 2);
                } else if (RIGHT_ZONE.contains(duckCenterPoint)) {
                    Imgproc.rectangle(displayMat, RIGHT_ZONE, RGB_GREEN, 2);
                }

            } else {
                // Uhhh....FIXME - no ducks
            }
        }



        // This makes sure we get output at the driver station - which will include the helpful
        // visual information we've added.

        return displayMat;
    }

    private Point centerPoint(final Rect boundingRect) {
        final Point topLeft = boundingRect.tl();
        final Point bottomRight = boundingRect.br();

        final Point centerPoint = new Point(
                topLeft.x + ((bottomRight.x - topLeft.x) / 2),
                topLeft.y + ((bottomRight.y - topLeft.y) / 2));

        return centerPoint;
    }
}
