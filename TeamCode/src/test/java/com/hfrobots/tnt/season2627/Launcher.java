package com.hfrobots.tnt.season2627;

import com.hfrobots.tnt.corelib.task.PeriodicTask;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher implements PeriodicTask {
    public static final int LAUNCH_VELOCITY = 9999;
    private final DcMotorEx launcherMotor;

    private State currentState = State.EMPTY;

    enum State {
        EMPTY,
        LOADED,
        GETTING_READY_TO_LAUNCH,
        READY_TO_LAUNCH,
        LAUNCHING
    };

    public Launcher(final HardwareMap hardwareMap) {
        this.launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
    }

    @Override
    public void periodicTask() {
        switch (currentState) {
            case EMPTY:
                // Detect or Told
                if (true) {
                    currentState = State.LOADED;
                    break;
                }
                break;
            case LOADED:
                // Requested to launch?
                if (true) {
                    currentState = State.GETTING_READY_TO_LAUNCH;
                    break;
                }
            case GETTING_READY_TO_LAUNCH:
                launcherMotor.setVelocity(LAUNCH_VELOCITY);
                launcherMotor.setPower(1.0D);

                // Are we ready to launch?
                if (checkReadyToLaunch()) {
                    currentState = State.READY_TO_LAUNCH;
                    break;

                }
            case READY_TO_LAUNCH:
                // check that feed requested?
                if (false) {
                    currentState = State.LAUNCHING;
                }

                break;
            case LAUNCHING:
                // Feed it, then go back to ready to launch
                currentState = State.READY_TO_LAUNCH;
                break;
        }
    }

    private boolean checkReadyToLaunch() {
        return (launcherMotor.getPower() != 0 && launcherMotor.getVelocity() >= LAUNCH_VELOCITY);
    }


}
