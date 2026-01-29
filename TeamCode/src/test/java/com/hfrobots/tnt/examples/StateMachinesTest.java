/*
 Copyright (c) 2025 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.examples;

import com.ftc9929.corelib.state.SequenceOfStates;
import com.ftc9929.corelib.state.State;
import com.google.common.testing.FakeTicker;
import com.hfrobots.tnt.fakes.FakeTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Before;
import org.junit.Test;

public class StateMachinesTest {
    private final FakeTelemetry fakeTelemetry = new FakeTelemetry();


    // This is a class that has code for a State machine state
    // that counts from 1 to 5, displaying it via telemetry
    static class CountToFiveState extends State {

        protected int currentCount = 0;

        protected CountToFiveState(final Telemetry telemetry) {
            // All State implementations must have a name
            // and access to telemetry (to report back to the drivers'
            // station
            super("Counting to five", telemetry);
        }

        /// Perform the logic for this state, return the next state which may be
        /// this state if we are to remain in this state.
        ///
        /// Called repeatedly in a loop by StateMachine (and the like)
        /// for every loop() call in the FTC SDK
        @Override
        public State doStuffAndGetNextState() {
            currentCount++;

            telemetry.addData("Current count", currentCount);

            if (currentCount == 5) {
                resetToStart();

                return nextState;
            }

            return this;
        }

        @Override
        public void resetToStart() {
            // Let's introduce a bug, instances of this state will not be reusable
        }
    }

    // This extends the behavior of CountToFiveState to provide
    // correct counting when it is reused
    static class ReentrantCountToFiveState extends CountToFiveState {
        protected ReentrantCountToFiveState(Telemetry telemetry) {
            super(telemetry);
        }

        @Override
        public void resetToStart() {
            // You can now see the reason for this overridable method
            // in the State class. It gives a common place for state
            // machine code to put everything needed to reset "to the
            // beginning", as well as the possibility for classes that
            // extend existing state machine code to reset themselves, and
            // then make sure that the parent class does as well.
            currentCount = 0;
        }
    }

    // Run this test, and watch the console output, you should see something like this:
    //
    // Telemetry: Current count 1
    // Telemetry: Current count 2
    // Telemetry: Current count 3
    // Telemetry: Current count 4
    // Telemetry: Current count 5
    // Telemetry: State machine Transition from com.hfrobots.tnt.examples.StateMachines$CountToFiveState@36804139 to null
    @Test
    public void countToFiveTest() {
        State countToFiveState = new CountToFiveState(fakeTelemetry);

        runStateMachine(countToFiveState);
    }

    // If you run this test, get ready to stop it in your IDE,
    // because it is an infinite loop, but also, it might not behave
    // the way you think - look at what the counting does!
    //
    // Note - our robot states even through re-used, never really
    //        have this endless looping problem, because they are usually
    //        of the form - wait for input -> do something -> wait for input
    //        where input could be from a human, or a sensor, or just the
    //        passing of time
    @Test
    public void countToFiveTwiceIncorrectlyTest() {
        State countToFiveState = new CountToFiveState(fakeTelemetry);

        // Let's attempt to count to five, twice, by running this state
        // immediately after the first one...
        countToFiveState.setNextState(countToFiveState);
        runStateMachine(countToFiveState);
    }

    // This code will also run forever, but it should count correctly
    @Test
    public void countToFiveTwiceCorrectlyTest() {
        State countToFiveState = new ReentrantCountToFiveState(fakeTelemetry);

        // Let's attempt to count to five, twice, by running this state
        // immediately after the first one...
        countToFiveState.setNextState(countToFiveState);
        runStateMachine(countToFiveState);
    }

    // This won't actually run, because of the reasons given below
    // but should show you how we use sequences to make things simpler
    // in auto, which is almost always do step 1, step 2, step 3 in order
    // once something has been detected/planned based on randomization
    public void exampleOfSequences() {
        SequenceOfStates sequence = new SequenceOfStates(new FakeTicker(), fakeTelemetry);
        State countToFiveState = new CountToFiveState(fakeTelemetry);
        State countToFiveStateAgain = new CountToFiveState(fakeTelemetry);

        // Adding states to a sequence takes care of
        // the keeping track of, what is the first state to give to the
        // state machine, and automatically setting the "next pointer"
        // from the previous state to the one being added:
        sequence.addSequential(countToFiveState);
        sequence.addSequential(countToFiveStateAgain);
    }

    // Our robot has code that does this with fancy logging and telemetry and such
    // but android logging is not really unit testable, and so we have to show
    // a simpler way to run this.

    // However, this is basically what the TNT core library StateMachine class does
    // in doOneStateLoop(), over and over again
    private void runStateMachine(final State startingState) {
        State currentState = startingState;

        // Keep asking a state to run and tell us which state
        // is next (it might be the same state!) - and finish
        // if there are no more state transitions (returns null)
        while (currentState != null) {
            State nextState = currentState.doStuffAndGetNextState();

            if (nextState != currentState) {
                fakeTelemetry.addData("State machine", "Transition from " + currentState + " to " + nextState);
            }

            currentState = nextState;
        }
    }

    @Before
    public void setupTests() {
        fakeTelemetry.setOutputTelemetry(true);
    }
}
