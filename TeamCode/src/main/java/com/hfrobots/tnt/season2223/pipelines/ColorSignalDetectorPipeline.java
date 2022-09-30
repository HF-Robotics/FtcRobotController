package com.hfrobots.tnt.season2223.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorSignalDetectorPipeline extends OpenCvPipeline {
    private final Scalar RGB_CYAN = new Scalar(4, 255, 255);

    private final Scalar RGB_MAGENTA = new Scalar(255, 10, 255);

    private final Scalar RGB_WHITE = new Scalar(255, 255, 255);

    private final ContourFinderPipeline cyanContourFinder;

    private final ContourFinderPipeline magentaContourFinder;

    private final ContourFinderPipeline blackContourFinder;

    private final Telemetry telemetry;

    private final Mat displayMat = new Mat();

    public ColorSignalDetectorPipeline(ContourFinderPipeline cyanContourFinder,
                                       ContourFinderPipeline magentaContourFinder,
                                       ContourFinderPipeline blackContourFinder,
                                       Telemetry telemetry) {
        this.cyanContourFinder = cyanContourFinder;
        this.magentaContourFinder = magentaContourFinder;
        this.blackContourFinder = blackContourFinder;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(displayMat);

        int numberOfCyanContours = 0;

        if (cyanContourFinder != null) {
            final Mat cyanMat = new Mat();
            final ArrayList<MatOfPoint> cyanContours;

            try {
                input.copyTo(cyanMat);
                cyanContours = cyanContourFinder.findContours(cyanMat);
                numberOfCyanContours = cyanContours.size();
            } finally {
                cyanMat.release(); // prevent native memory leak
            }

            Imgproc.drawContours(displayMat, cyanContours, -1, RGB_CYAN, 2);
        }

        if (numberOfCyanContours == 1) {
            telemetry.addLine("Found cyan signal");
        } else {
            telemetry.addLine("No signal detected");
        }

        return displayMat; // show on the DS, what the camera actually saw, with contours highlighted
    }
}
