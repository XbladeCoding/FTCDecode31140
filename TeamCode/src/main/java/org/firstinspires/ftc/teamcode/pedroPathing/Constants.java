package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.spec.ECField;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.85, 0.2, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.2,0.05,0.01,0.6,0))
            .forwardZeroPowerAcceleration(-25.28540967)
            .lateralZeroPowerAcceleration(-76.33987998)
            .mass(10.6);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(50.31245042)
            .yVelocity(41.74711011)
            .maxPower(1)
            .rightFrontMotorName("leftRearDrive")
            .rightRearMotorName("leftFrontDrive")
            .leftRearMotorName("rightFrontDrive")
            .leftFrontMotorName("rightRearDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 300, 0.65, 0.75);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.0019792)
            .strafeTicksToInches(.0019792)
            .turnTicksToInches(.0019792)
            .leftPodY(5.96)
            .rightPodY(-6.04)
            .strafePodX(5.4375)
            .leftEncoder_HardwareMapName("rightRearDrive")
            .rightEncoder_HardwareMapName("leftRearDrive")
            .strafeEncoder_HardwareMapName("leftFrontDrive")
            .leftEncoderDirection(Encoder.FORWARD) // REVERSE
            .rightEncoderDirection(Encoder.REVERSE) // REVERSE
            .strafeEncoderDirection(Encoder.REVERSE); // FORWARD
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
