package com.hfrobots.tnt.season2627;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BiobuzzPedroConstants {
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("rightFrontDriveMotor")
            .rightRearMotorName("rightRearDriveMotor")
            .leftRearMotorName("leftRearDriveMotor")
            .leftFrontMotorName("leftFrontDriveMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(77.192)
            .yVelocity(53.1516);

    public static FollowerConstants followerConstants = new FollowerConstants()
            //.mass(14)
            .forwardZeroPowerAcceleration(-25.518)
            .lateralZeroPowerAcceleration(-94.1348);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .leftPodY(5.5)
            .rightPodY(-5.5)
            .strafePodX(-1.0)
            .leftEncoder_HardwareMapName("leftFrontDriveMotor") // red
            .rightEncoder_HardwareMapName("rightRearDriveMotor") // yellow
            .strafeEncoder_HardwareMapName("rightFrontDriveMotor") // green
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.002015282559)
            .strafeTicksToInches(0.001937436932)
            .turnTicksToInches( 0.00034955);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
