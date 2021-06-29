package com.acmerobotics.roadrunner.quickstart.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.hfrobots.tnt.corelib.drive.roadrunner.RoadRunnerMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerMecanumDrive drive = new RoadRunnerMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
