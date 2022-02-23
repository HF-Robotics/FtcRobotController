package com.hfrobots.tnt.corelib.drive.mecanum;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.state.RepeatingLambdaState;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import lombok.Builder;
import lombok.NonNull;

@Builder
public class TrajectoryAwareSequenceOfStates extends SequenceOfStates {
    private final RoadRunnerMecanumDriveBase driveBase;

    private final Telemetry telemetry;

    private final Ticker ticker;

    public void addTrajectory(@NonNull final String name, Function<TrajectoryBuilder, TrajectoryBuilder> trjaectoryProvider) {
        SimpleTrajectoryState state = new SimpleTrajectoryState(name, trjaectoryProvider, telemetry, driveBase, ticker);
        addSequential(state);
    }

    public void addRepeatingState(@NonNull final String name, @NonNull final Function<State, Boolean> stateLogic) {
        addSequential(new RepeatingLambdaState(name, stateLogic, telemetry));
    }

    public static void main(String[] args) {
        TrajectoryAwareSequenceOfStatesBuilder tasb = TrajectoryAwareSequenceOfStates.builder().telemetry(null).driveBase(null).ticker(null);

        TrajectoryAwareSequenceOfStates steps = tasb.build();

        Constants.Alliance currentAlliance = Constants.Alliance.BLUE;

        // Simply drive forward (or any other trajectory)
        steps.addTrajectory("Drive forward 1 inch",
                (t) -> t.forward(1));

        // Move differently depending on external state
        steps.addTrajectory("Strafe based on alliance",
                (t) -> {
                    if (currentAlliance == Constants.Alliance.BLUE) {
                        return t.strafeLeft(15);
                    } else {
                        return t.strafeRight(15);
                    }
                });

        DistanceSensor distanceSensor = null;

        // Do something until a condition happens, then go on to the
        // next thing...
        steps.addRepeatingState("Repeat something",
                (myself) -> {
                    if (distanceSensor.getDistance(DistanceUnit.INCH) < 28) {
                        return false;
                    }

                    return true;
                });
    }
}
