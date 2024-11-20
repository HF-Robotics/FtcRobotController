package com.hfrobots.tnt.corelib.drive.roadrunner1.quickstart;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

public interface Localizer {
    Twist2dDual<Time> update();
}
