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
            .maxPower(1.0)
            .rightFrontMotorName("rightFrontDriveMotor")
            .rightRearMotorName("rightRearDriveMotor")
            .leftRearMotorName("leftRearDriveMotor")
            .leftFrontMotorName("leftFrontDriveMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(50.969)
            .yVelocity(32.348);
    ;

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .leftPodY(3.54)
            .rightPodY(-3.54)
            .strafePodX(-7.6)
            /*
            <Motor name="leftFrontDriveMotor"  port="0" /> <!-- Red    -->
            <Motor name="leftRearDriveMotor"   port="1" /> <!-- Orange -->
            <Motor name="rightFrontDriveMotor" port="2" /> <!-- Yellow -->
            <Motor name="rightRearDriveMotor"  port="3" /> <!-- Green  -->
             */
            .leftEncoder_HardwareMapName("leftFrontDriveMotor") // red
            .rightEncoder_HardwareMapName("rightRearDriveMotor") // yellow
            .strafeEncoder_HardwareMapName("rightFrontDriveMotor") // green
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.002933)
            .strafeTicksToInches(0.002957)
            .turnTicksToInches( 0.0029474);


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-35.551)
            .lateralZeroPowerAcceleration(-85.506)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
