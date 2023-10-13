/*
 Copyright (c) 2023 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2324;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.Log;

import com.google.common.collect.Lists;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import lombok.Builder;
import lombok.Value;

public class SpikeStripDetector implements VisionProcessor {

    private final double xResolutionPx = 640;

    private final int zoneWidthPx = (int)(xResolutionPx / 3);

    private final int zoneStartY = 130;

    private final int zoneHeight = 300 - zoneStartY;

    private final Rect leftZone = new Rect(0, zoneStartY, zoneWidthPx, zoneHeight);

    private final  Rect centerZone = new Rect(zoneWidthPx + 1, zoneStartY, zoneWidthPx, zoneHeight);

    public final Rect rightZone = new Rect(2 * zoneWidthPx + 1, zoneStartY, zoneWidthPx, zoneHeight);

    enum DetectedSpikeStrip { LEFT, CENTER, RIGHT, UNKNOWN }

    // Start with unknown, because until we start the pipeline
    // we do not know what signal the robot is seeing
    private final AtomicReference<DetectedSpikeStrip> detectedSpikeStripRef
            = new AtomicReference<>(DetectedSpikeStrip.UNKNOWN);

    private boolean recordDetection = false;

    private final Mat bgrMat = new Mat();

    private final GripBluePropPipeline bluePropPipeline = new GripBluePropPipeline();

    private final GripRedPropPipeline redPropPipeline = new GripRedPropPipeline();

    private boolean useRedPipeline = true;

    private boolean useBluePipeline = !useRedPipeline;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public void useRedPipeline() {
        useRedPipeline = true;
        useBluePipeline = false;
    }

    public void useBluePipeline() {
        useBluePipeline = true;
        useRedPipeline = false;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // OpenCV expects frames in BGR for things like HSV to work
        // EasyOpenCV sends in RGBA, must convert them before doing anything else
        Imgproc.cvtColor(frame, bgrMat, Imgproc.COLOR_RGBA2BGR);

        final ArrayList<MatOfPoint> filteredContours;

        if (useBluePipeline) {
            bluePropPipeline.process(bgrMat);
            filteredContours = bluePropPipeline.filterContoursOutput();
            Log.d("TNT", "Used blue pipeline, found " + filteredContours.size());
        } else {
            // FIXME: We have two pipelines: bluePropPipeline and redPropPipeline, fix this
            // to choose the red one when that one is chosen?
            redPropPipeline.process(bgrMat);
            filteredContours = redPropPipeline.filterContoursOutput();
            Log.d("TNT", "Used red pipeline, found " + filteredContours.size());
        }

        ArrayList<Rect> boundingBoxes = Lists.newArrayList();

        for (MatOfPoint contour : filteredContours) {
            final Rect boundingRect = Imgproc.boundingRect(contour);
            boundingBoxes.add(boundingRect);

            final Point propCenterPoint = centerPoint(boundingRect);

            // How do we deal with contours outside of the detection zone
            // "erasing" knowledge of ones that were detected inside?
            if (leftZone.contains(propCenterPoint)) {
                detectedSpikeStripRef.set(DetectedSpikeStrip.LEFT);
            } else if (centerZone.contains(propCenterPoint)) {
                detectedSpikeStripRef.set(DetectedSpikeStrip.CENTER);
            } else if (rightZone.contains(propCenterPoint)) {
                detectedSpikeStripRef.set(DetectedSpikeStrip.RIGHT);
            } else {
                detectedSpikeStripRef.set(DetectedSpikeStrip.UNKNOWN);
            }
        }

        return DetectionData.builder().detectedSpikeStrip(getDetectedSpikeStrip())
                .filteredBoundingBoxes(boundingBoxes).build();
    }

    @Value
    @Builder
    static class DetectionData {
        private ArrayList<MatOfPoint> contours;

        private ArrayList<MatOfPoint> filteredContours;

        private ArrayList<Rect> filteredBoundingBoxes;

        private DetectedSpikeStrip detectedSpikeStrip;
    }

    private Point centerPoint(final Rect boundingRect) {
        final Point topLeft = boundingRect.tl();
        final Point bottomRight = boundingRect.br();

        final Point centerPoint = new Point(
                topLeft.x + ((bottomRight.x - topLeft.x) / 2),
                topLeft.y + ((bottomRight.y - topLeft.y) / 2));

        return centerPoint;
    }

    @Override
    public void onDrawFrame(final Canvas canvas, final int onscreenWidth, final int onscreenHeight,
                            final float scaleBmpPxToCanvasPx, final float scaleCanvasDensity,
                            final Object userContext) {
        final Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 8);

        final Paint nonSelectedPaint = new Paint(selectedPaint); nonSelectedPaint.setColor(Color.YELLOW);
        nonSelectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        final android.graphics.Rect leftZoneRect = convertToAndroidRect(leftZone, scaleBmpPxToCanvasPx);
        final android.graphics.Rect centerZoneRect = convertToAndroidRect(centerZone, scaleBmpPxToCanvasPx);
        final android.graphics.Rect rightZoneRect = convertToAndroidRect(rightZone, scaleBmpPxToCanvasPx);

        drawZones(canvas, nonSelectedPaint, leftZoneRect, centerZoneRect, rightZoneRect);

        DetectionData detectionData = (DetectionData)userContext;
        ArrayList<Rect> boundingBoxes = detectionData.getFilteredBoundingBoxes();

        if (boundingBoxes != null) {
            for (Rect boundingBox : boundingBoxes) {
                final android.graphics.Rect boundingBoxRect = convertToAndroidRect(boundingBox, scaleBmpPxToCanvasPx);
                canvas.drawRect(boundingBoxRect, nonSelectedPaint);
            }
        }

        DetectedSpikeStrip detected = detectionData.getDetectedSpikeStrip();

        switch (detected) {
            case LEFT:
                canvas.drawRect(leftZoneRect, selectedPaint);
                break;
            case CENTER:
                canvas.drawRect(centerZoneRect, selectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(rightZoneRect, selectedPaint);
                break;
            case UNKNOWN:
                drawZones(canvas, nonSelectedPaint, leftZoneRect, centerZoneRect, rightZoneRect);
                break;
        }
    }

    private static void drawZones(Canvas canvas, Paint nonSelectedPaint, android.graphics.Rect leftZoneRect, android.graphics.Rect centerZoneRect, android.graphics.Rect rightZoneRect) {
        canvas.drawRect(leftZoneRect, nonSelectedPaint);
        canvas.drawRect(centerZoneRect, nonSelectedPaint);
        canvas.drawRect(rightZoneRect, nonSelectedPaint);
    }

    void recordDetections() {
        recordDetection = true;
    }

    DetectedSpikeStrip getDetectedSpikeStrip() {
        return detectedSpikeStripRef.get();
    }

    private android.graphics.Rect convertToAndroidRect(final Rect rect, final float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    private android.graphics.Path convertContourToAndroidPath(final MatOfPoint contour,
                                                              final float scaleBmpPxToCanvasPx) {
        Point[] points = contour.toArray();

        if (points.length > 0) {
            final Path path = new Path();
            path.moveTo((float) (points[0].x * scaleBmpPxToCanvasPx),
                (float) (points[0].y * scaleBmpPxToCanvasPx));

            for (int i = 1; i < points.length; i++) {
                //path.lineTo((float) (points[i].x * scaleBmpPxToCanvasPx),
                //        (float) (points[i]).y * scaleBmpPxToCanvasPx);
            }

            path.close();

            return path;
        }

        return null;
    }
}
