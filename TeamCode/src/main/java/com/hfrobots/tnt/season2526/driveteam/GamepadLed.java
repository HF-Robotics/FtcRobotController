package com.hfrobots.tnt.season2526.driveteam;

import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Used so we don't need to pass full-fledged Gamepads around
 */
public class GamepadLed {
    private final Gamepad gamepad;

    public GamepadLed(final Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void setColor(final int red, final int green, final int blue, final int durationMs) {
        if (this.gamepad != null) {
            this.gamepad.setLedColor(red, green, blue, durationMs);
        }
    }
}
