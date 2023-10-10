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

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class SpikeStripDetector implements VisionProcessor {

    private final double xResolutionPx = 640;

    private final int zoneWidthPx = (int)(xResolutionPx / 3);

    private final Rect leftZone = new Rect(0, 0, zoneWidthPx, 480);

    private final  Rect centerZone = new Rect(zoneWidthPx + 1, 0, zoneWidthPx, 480);

    public final Rect rightZone = new Rect(2 * zoneWidthPx + 1, 0, zoneWidthPx, 480);

    enum DetectedSpikeStrip { LEFT, CENTER, RIGHT, UNKNOWN }

    // Start with unknown, because until we start the pipeline
    // we do not know what signal the robot is seeing
    private final AtomicReference<DetectedSpikeStrip> detectedSpikeStripRef
            = new AtomicReference<>(DetectedSpikeStrip.UNKNOWN);

    private boolean recordDetection = false;

    private final Mat bgrMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // OpenCV expects frames in BGR for things like HSV to work
        // EasyOpenCV sends in RGBA, must convert them before doing anything else
        Imgproc.cvtColor(frame, bgrMat, Imgproc.COLOR_RGBA2BGR);

        return getDetectedSpikeStrip();
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
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        final Paint nonSelectedPaint = new Paint(selectedPaint); nonSelectedPaint.setColor(Color.YELLOW);

        final Paint leftPaint = new Paint(selectedPaint);
        leftPaint.setColor(Color.RED);
        final Paint rightPaint = new Paint(selectedPaint);
        rightPaint.setColor(Color.CYAN);

        final android.graphics.Rect leftZoneRect = convertToAndroidRect(leftZone, scaleBmpPxToCanvasPx);
        final android.graphics.Rect centerZoneRect = convertToAndroidRect(centerZone, scaleBmpPxToCanvasPx);
        final android.graphics.Rect rightZoneRect = convertToAndroidRect(rightZone, scaleBmpPxToCanvasPx);

        canvas.drawRect(leftZoneRect, leftPaint);
        canvas.drawRect(centerZoneRect, nonSelectedPaint);
        canvas.drawRect(rightZoneRect, rightPaint);

        DetectedSpikeStrip detected = getDetectedSpikeStrip();

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
        }

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
