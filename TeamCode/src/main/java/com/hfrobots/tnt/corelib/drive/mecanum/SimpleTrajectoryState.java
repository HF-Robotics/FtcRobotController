package com.hfrobots.tnt.corelib.drive.mecanum;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.google.common.base.Ticker;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

public class SimpleTrajectoryState extends TrajectoryFollowerState {

    private final Function<TrajectoryBuilder, TrajectoryBuilder> trajectoryProvider;

    protected SimpleTrajectoryState(@NonNull final String name, @NonNull Function<TrajectoryBuilder, TrajectoryBuilder> trajectoryProvider,
                                    Telemetry telemetry,
                                    RoadRunnerMecanumDriveBase driveBase,
                                    Ticker ticker) {
        super(name, telemetry, driveBase, ticker, TimeUnit.SECONDS.toMillis(30) /* FIXME */);
        this.trajectoryProvider = trajectoryProvider;
    }

    @Override
    protected Trajectory createTrajectory() {
        return trajectoryProvider.apply(driveBase.trajectoryBuilder()).build();
    }
}
