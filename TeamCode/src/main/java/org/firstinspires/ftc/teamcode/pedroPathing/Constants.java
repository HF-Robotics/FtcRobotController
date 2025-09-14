package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDriveMotor")
            .rightRearMotorName("rightRearDriveMotor")
            .leftRearMotorName("leftRearDriveMotor")
            .leftFrontMotorName("leftFrontDriveMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(83.30172521)
            .yVelocity(68.403580003);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001997896808)
            .strafeTicksToInches( .001950693882)
            .turnTicksToInches(   .002009086694)
            .leftPodY(2.44)
            .rightPodY(-1.65)
            .strafePodX(-6.06)
            .leftEncoder_HardwareMapName("rightFrontDriveMotor")
            .rightEncoder_HardwareMapName("leftRearDriveMotor")
            .strafeEncoder_HardwareMapName("rightRearDriveMotor")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.20)
            .forwardZeroPowerAcceleration(-29.659244009)
            .lateralZeroPowerAcceleration(-41.66984);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
