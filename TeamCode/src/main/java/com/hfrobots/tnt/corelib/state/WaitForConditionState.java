/*
 Copyright (c) 2024 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.corelib.state;

import static com.ftc9929.corelib.Constants.LOG_TAG;

import android.util.Log;

import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchTimeoutSafetyState;
import com.google.common.base.Ticker;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Callable;

import lombok.Builder;
import lombok.NonNull;
import lombok.SneakyThrows;

@SuppressWarnings("unused")
public class WaitForConditionState extends StopwatchTimeoutSafetyState {
    final private Callable<Boolean> condition;

    @Builder
    private WaitForConditionState(@NonNull String name,
                                  final Callable<Boolean> condition,
                                  Telemetry telemetry,
                                  @NonNull Ticker ticker,
                                  long safetyTimeoutMillis) {
        super(name, telemetry, ticker, safetyTimeoutMillis);
        this.condition = condition;
    }

    @Override
    @SneakyThrows
    public State doStuffAndGetNextState() {
        if (isTimedOut()) {
            resetToStart();
            Log.e(LOG_TAG, "Timed out waiting for condition, moving to next state");

            return nextState;
        }

        if (condition.call()) {
            Log.i(LOG_TAG, "Condition met, moving to next state");

            return nextState;
        }

        return this;
    }
}
