/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import lombok.AllArgsConstructor;
import lombok.Builder;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Builder
@AllArgsConstructor
public class EasyOpenCvPipelineAndCamera {
    private final OpenCvPipeline openCvPipeline;

    private final HardwareMap hardwareMap;

    private final Telemetry telemetry;

    private OpenCvCamera cvCamera;

    public void createAndRunPipeline() {
        try {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            // Webcam...
            cvCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

            /*
             * Specify the image processing pipeline we wish to invoke upon receipt
             * of a frame from the camera. Note that switching pipelines on-the-fly
             * (while a streaming session is in flight) *IS* supported.
             */
            cvCamera.setPipeline(openCvPipeline);

            // This is a good proxy for "things are working" that is visible in the "camera stream"
            // view on the drivers' station, now that we no longer have displays on-robot.
            cvCamera.showFpsMeterOnViewport(true);

            cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    /*
                     * Tell the webcam to start streaming images to us! Note that you must make sure
                     * the resolution you specify is supported by the camera. If it is not, an exception
                     * will be thrown.
                     *
                     * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                     * supports streaming from the webcam in the uncompressed YUV image format. This means
                     * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                     * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                     *
                     * Also, we specify the rotation that the webcam is used in. This is so that the image
                     * from the camera sensor can be rotated such that it is always displayed with the image upright.
                     * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                     * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                     * away from the user.
                     */
                    cvCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                    telemetry.addData("CV", "Camera failed: " + errorCode);
                    Log.e(LOG_TAG, "Failed to open webcam, error: " + errorCode);
                }
            });
        } catch (Throwable t) {
            Log.e(LOG_TAG, "Failed to start OpenCV Pipeline", t);
        }
    }

    public void pauseViewport() {
        if (cvCamera != null) {
            cvCamera.pauseViewport();
        }
    }

    public void resumeViewport() {
        if (cvCamera != null) {
            cvCamera.resumeViewport();
        }
    }

    public void stopStreaming() {
        cvCamera.stopStreaming();
    }

    public void closeCameraDeviceAsync() {
        cvCamera.closeCameraDeviceAsync(() -> Log.i(LOG_TAG, "Camera has been closed"));
    }
}
