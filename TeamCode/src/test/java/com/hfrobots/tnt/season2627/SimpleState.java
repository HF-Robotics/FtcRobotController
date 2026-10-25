package com.hfrobots.tnt.season2627;

import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Callable;

import lombok.NonNull;
import lombok.SneakyThrows;

public class SimpleState extends StopwatchTimeoutSafetyState {
    private final Callable<State> logic;

    public SimpleState(@NonNull Callable<State> logic,
                       @NonNull final String name,
                       final Telemetry telemetry,
                       @NonNull final Ticker ticker,
                       long safetyTimeoutMillis) {
        super(name, telemetry, ticker, safetyTimeoutMillis);
        this.logic = logic;
    }

    @Override
    @SneakyThrows
    public State doStuffAndGetNextState() {
        return logic.call();
    }

    public static void main(String[] args) {
        new SimpleState(() -> { return null;}, "foo", null, Ticker.systemTicker(), 5);
    }
}
