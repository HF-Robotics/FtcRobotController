/*
 Copyright (c) 2023 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.corelib.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import lombok.AllArgsConstructor;

// So we don't have to pass around Gamepads that might get used incorrectly, just to send rumble
// signals to them
@AllArgsConstructor
public class RumbleTarget {
    private final Gamepad gamepad;

    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }

    public void rumble(double rumble1, double rumble2, int durationMs) {
        gamepad.rumble(rumble1, rumble2, durationMs);
    }

    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

    public void stopRumble() {
        gamepad.stopRumble();
    }

    public void runRumbleEffect(Gamepad.RumbleEffect effect) {
        gamepad.runRumbleEffect(effect);
    }

}
