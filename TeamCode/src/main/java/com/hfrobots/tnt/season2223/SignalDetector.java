/*
 Copyright (c) 2022 The Tech Ninja Team (https://ftc9929.com)

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

package com.hfrobots.tnt.season2223;

// This is the "contract" between autonomous and all ways we have come up with to detect
// our custom signal sleeve (colors, apriltags, hybrids of the two).
//
// It separates concerns of detecting and what to do based on the detected signal, especially
// the unknown case. SignalDetectors need only know how to detect the signal sleeve, a state
// in autonomous will decide what to do based on the detected (or undetected) signal.

import lombok.NonNull;

public interface SignalDetector {
    enum DetectedSignal { ORIENTATION_A, ORIENTATION_B, ORIENTATION_C, UNKNOWN }

    @NonNull
    DetectedSignal getDetectedSignal();
}
