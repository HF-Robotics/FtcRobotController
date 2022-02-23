package com.hfrobots.tnt.corelib.state;

import com.ftc9929.corelib.state.State;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BiFunction;

public class RepeatingLambdaState extends State {
    private final Function<State, Boolean> lambda;

    public RepeatingLambdaState(final String name, final Function<State, Boolean> lambda, Telemetry telemetry) {
        super("", telemetry);
        this.lambda = lambda;
    }

    @Override
    public State doStuffAndGetNextState() {
        boolean shouldRepeat = lambda.apply(this);

        if (shouldRepeat) {
            return this;
        }

        return nextState;
    }

    @Override
    public void resetToStart() {

    }
}
