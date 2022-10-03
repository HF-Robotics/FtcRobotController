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

package com.hfrobots.tnt.util.opencv;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ContourUtils {

    public static final Scalar RGB_GREEN = new Scalar(0, 255, 0);

    public static void labelContour(Mat displayMat, MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);

        double area = Imgproc.contourArea(contour);

        String dimensionLabel = boundingRect.width + " x " + boundingRect.height + " (" + area + ")";

        Imgproc.line(displayMat,
                new Point(boundingRect.x + (boundingRect.width /2),
                        boundingRect.y + (boundingRect.height / 2)),
                new Point(boundingRect.x + boundingRect.width,
                        boundingRect.y + boundingRect.height), RGB_GREEN, 2);

        Imgproc.putText(displayMat, dimensionLabel, new Point(boundingRect.x + boundingRect.width + 10,
                boundingRect.y + boundingRect.height + 10),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, RGB_GREEN, 2);

        String locationLabel = "(" + boundingRect.x + ", " + boundingRect.y + ")";

        Imgproc.putText(displayMat, locationLabel, new Point(boundingRect.x - 10,
                boundingRect.y - 10),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, RGB_GREEN, 2);

    }
}
