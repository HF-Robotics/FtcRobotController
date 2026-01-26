package com.hfrobots.tnt.season2526;

import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.state.State;
import com.google.common.base.Ticker;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.NonNull;

public interface GenevaCarousel {
    void doAutoIntakeStuff();

    void resetAutoIntake();

    boolean isBusyIndexing();

    void nextIndexForIntake();

    boolean notDoneAdvancing();

    boolean isInLaunchPosition();

    void nextIndexForLaunch();

    void manuallyAdjust(RangeInput carouselThrottle, boolean unsafeIsPressed);

    State nextLaunchIndexState(Telemetry telemetry, @NonNull Ticker ticker, boolean skipIfInPosition);

    State nextIntakeIndexState(Telemetry telemetry, @NonNull Ticker ticker, boolean skipIfInPosition);
}
