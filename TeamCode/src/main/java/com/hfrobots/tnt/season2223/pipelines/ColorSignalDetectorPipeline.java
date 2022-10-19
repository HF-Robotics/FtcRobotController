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

package com.hfrobots.tnt.season2223.pipelines;

import com.google.common.base.Stopwatch;
import com.hfrobots.tnt.season2223.SignalDetector;
import com.hfrobots.tnt.util.opencv.ContourUtils;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ColorSignalDetectorPipeline extends OpenCvPipeline implements SignalDetector {
    private final Scalar RGB_CYAN = new Scalar(4, 255, 255);

    private final Scalar RGB_MAGENTA = new Scalar(255, 10, 255);

    private final Scalar RGB_WHITE = new Scalar(255, 255, 255);

    private final ContourFinderPipeline cyanContourFinder;

    private final ContourFinderPipeline pinkContourFinder;

    private final ContourFinderPipeline blackContourFinder;

    private final Telemetry telemetry;

    private DescriptiveStatistics stats = new DescriptiveStatistics(1000);

    private final Mat displayMat = new Mat();

    private final Mat bgrMat = new Mat();

    private final Mat cyanMat = new Mat();

    private final Mat pinkMat = new Mat();

    // Start with unknown, because until we start the pipeline
    // we do not know what signal the robot is seeing
    private final AtomicReference<DetectedSignal> detectedSignalRef
            = new AtomicReference<>(DetectedSignal.UNKNOWN);

    public ColorSignalDetectorPipeline(ContourFinderPipeline cyanContourFinder,
                                       ContourFinderPipeline pinkContourFinder,
                                       ContourFinderPipeline blackContourFinder,
                                       Telemetry telemetry) {
        this.cyanContourFinder = cyanContourFinder;
        this.pinkContourFinder = pinkContourFinder;
        this.blackContourFinder = blackContourFinder;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Stopwatch timer = Stopwatch.createStarted();

        // OpenCV expects frames in BGR for things like HSV to work
        // EasyOpenCV sends in RGBA, must convert them before doing anything else
        Imgproc.cvtColor(input, bgrMat, Imgproc.COLOR_RGBA2BGR);

        input.copyTo(displayMat);

        int numberOfCyanContours = 0;

        if (cyanContourFinder != null) {
            final ArrayList<MatOfPoint> cyanContours;

            bgrMat.copyTo(cyanMat);
            cyanContours = cyanContourFinder.findContours(cyanMat);
            numberOfCyanContours = cyanContours.size();

            for (MatOfPoint contour : cyanContours) {
                ContourUtils.labelContour(displayMat, contour);
            }

            Imgproc.drawContours(displayMat, cyanContours, -1, RGB_CYAN, 2);
        }

        // START HERE
        int numberOfPinkContours = 0;

        if (pinkContourFinder != null) {
            final ArrayList<MatOfPoint> pinkContours;

            bgrMat.copyTo(pinkMat);
            pinkContours = pinkContourFinder.findContours(pinkMat);
            numberOfPinkContours = pinkContours.size();

            for (MatOfPoint contour : pinkContours) {
                ContourUtils.labelContour(displayMat, contour);
            }

            Imgproc.drawContours(displayMat, pinkContours, -1, RGB_CYAN, 2);
        }

        // STOP HERE

        long pipelineTimeMicros = timer.elapsed(TimeUnit.MICROSECONDS);

        if (numberOfCyanContours == 1) {
            telemetry.addLine("Found cyan signal");

            detectedSignalRef.set(DetectedSignal.ORIENTATION_C);

            stats.addValue(pipelineTimeMicros);
        } else if (numberOfPinkContours == 1) {
            telemetry.addLine("Found pink signal");

            detectedSignalRef.set(DetectedSignal.ORIENTATION_A);

            stats.addValue(pipelineTimeMicros);
        } else {
            detectedSignalRef.set(DetectedSignal.UNKNOWN);
            telemetry.addLine("No signal detected");
        }

        telemetry.addLine("P95 time (ms)" + stats.getPercentile(95) / 1000);

        return displayMat; // show on the DS, what the camera actually saw, with contours highlighted
    }

    @Override
    public DetectedSignal getDetectedSignal() {
        return detectedSignalRef.get();
    }
}
