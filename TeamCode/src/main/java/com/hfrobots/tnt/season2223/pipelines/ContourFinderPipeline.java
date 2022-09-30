package com.hfrobots.tnt.season2223.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;

public interface ContourFinderPipeline {
    ArrayList<MatOfPoint> findContours(Mat source0);
}
