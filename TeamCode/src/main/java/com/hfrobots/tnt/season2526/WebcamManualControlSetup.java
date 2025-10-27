package com.hfrobots.tnt.season2526;

import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StateMachine;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.hfrobots.tnt.season2324.Shared;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

import lombok.Getter;

public class WebcamManualControlSetup implements PeriodicTask {
    private final VisionPortal visionPortal;

    private final StateMachine stateMachine;

    @Getter
    private boolean cameraIsSetup;

    public WebcamManualControlSetup(final VisionPortal visionPortal,
                                    final Telemetry telemetry,
                                    final Ticker ticker) {
        this.visionPortal = visionPortal;

        WaitForCameraStreamingState waitForStreamingState = new WaitForCameraStreamingState(telemetry);
        SetExposureState setExposureState = new SetExposureState(telemetry);
        SetGainState setGainState = new SetGainState(telemetry);

        SequenceOfStates states = new SequenceOfStates(ticker, telemetry);
        states.addSequential(waitForStreamingState);
        states.addSequential(setExposureState);
        states.addSequential(setGainState);
        states.addRunnableStep("Set camera enabled", () -> cameraIsSetup = true);

        stateMachine = new StateMachine(telemetry);
        stateMachine.addSequence(states);
    }

    @Override
    public void periodicTask() {
        Shared.withBetterErrorHandling(stateMachine::doOneStateLoop);
    }

    class WaitForCameraStreamingState extends State {

        protected WaitForCameraStreamingState(Telemetry telemetry) {
            super("waiting for camera", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                return this;
            }

            return nextState;
        }

        @Override
        public void resetToStart() {

        }
    }

    class SetExposureState extends State {

        protected SetExposureState(Telemetry telemetry) {
            super("Set exposure", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);

                    return this;
                }

                if (exposureControl.getExposure(TimeUnit.MILLISECONDS) != 6) {
                    exposureControl.setExposure((long) 6, TimeUnit.MILLISECONDS);

                    return this;
                }

                // Exposure has been set
                return nextState;

            }

            return this;
        }

        @Override
        public void resetToStart() {

        }
    }

    class SetGainState extends State {

        protected SetGainState(Telemetry telemetry) {
            super("Set gain", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

                if (gainControl.getGain() != 250) {
                    gainControl.setGain(250);

                    return this;
                }

                // Gain has been set
                return nextState;

            }

            return this;
        }

        @Override
        public void resetToStart() {

        }
    }
}
