
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;

import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.Path;

import com.pedropathing.paths.PathChain;

import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.util.ElapsedTime;





@Autonomous(name = "blueauton", group = "auton")

public class blueauton extends OpMode {



    // Robot motor initialization



//    public DcMotor  leftFrontDrive = null;

//    public DcMotor  rightFrontDrive = null;

//    public DcMotor  leftRearDrive = null;

//    public DcMotor  rightRearDrive = null;

    public CRServo  intakeServo = null;

    public DcMotor  leftConveyor = null;

    public DcMotor  rightConveyor = null;

    public DcMotor  leftLaunch = null;

    public DcMotor  rightLaunch = null;



    public VoltageSensor myVoltageSensor;





    // Odometry initialization

    private Follower follower;

    private Timer pathTimer, opmodeTimer, shootTimer, waitTimer;



    private int pathState;



    private final Pose startPose = new Pose(30.0, 135, Math.toRadians(90.0));

    private final Pose scorePose = new Pose(48.0, 96, Math.toRadians(135.0));

    private final Pose pickupPose1 = new Pose(48.0, 86, Math.toRadians(0.0));

    private final Pose pickupMidPose1 = new Pose(36.0, 86, Math.toRadians(0.0));

    private final Pose pickupMid2Pose1 = new Pose(30.0, 86, Math.toRadians(0.0));

    private final Pose pickupEndPose1 = new Pose(18.0, 86, Math.toRadians(0.0));

    private final Pose pickupPose2 = new Pose(48.0, 62, Math.toRadians(0.0));

    private final Pose pickupMidPose2 = new Pose(34.0, 62, Math.toRadians(0.0));

    private final Pose pickupMid2Pose2 = new Pose(24.0, 62, Math.toRadians(0.0));

    private final Pose pickupEndPose2 = new Pose(12.0, 62, Math.toRadians(0.0));

    private final Pose pickupPose3 = new Pose(48.0, 36, Math.toRadians(0.0));

    private final Pose pickupEndPose3 = new Pose(12.0, 36, Math.toRadians(0.0));

    private final Pose endPose = new Pose(36.0, 84, Math.toRadians(90.0));




    private Path scorePreload;

    private PathChain grabPickup1a, grabPickup1m, grabPickup1m2, grabPickup1b, scorePickup1, grabPickup2a, grabPickup2m, grabPickup2m2, grabPickup2b, scorePickup2, grabPickup3a, grabPickup3b, scorePickup3, end;





    // Shoot power!!

    private double shootConstant;



    private final double speedMovePercent = 0.3;



    private final double prepShootTime = 1.5, shootTime = 2.0, stabilizeTime = 2.0;

    private final double shoot2Time = 0.12;

    private final double shoot3Time = 0.8;





    public void prepShoot() {

        leftLaunch.setPower(shootConstant);

        rightLaunch.setPower(shootConstant);

    }



    public void shoot() {

        if (pathTimer.getElapsedTimeSeconds() < shoot2Time) {

            leftLaunch.setPower(shootConstant);

            rightLaunch.setPower(shootConstant);

        } else if (pathTimer.getElapsedTimeSeconds() < shoot3Time) {

            leftLaunch.setPower(shootConstant+0.08);

            rightLaunch.setPower(shootConstant+0.08);

        } else {

            leftLaunch.setPower(shootConstant);

            rightLaunch.setPower(shootConstant);

        }

    }





    public void intake() {

        intakeServo.setPower(1.0);

        leftConveyor.setPower(1.0);

        rightConveyor.setPower(1.0);

    }



    public void slowIntake() {

        intakeServo.setPower(1.0);

        leftConveyor.setPower(0.3);

        rightConveyor.setPower(0.3);

    }



    public void reverseIntake() {

        intakeServo.setPower(0);

        leftConveyor.setPower(-0.5);

        rightConveyor.setPower(-0.5);

        leftLaunch.setPower(-0.1);

        rightLaunch.setPower(-0.1);

    }



    public void stopShooting() {

        intakeServo.setPower(0);

        leftConveyor.setPower(0);

        rightConveyor.setPower(0);

        leftLaunch.setPower(0);

        rightLaunch.setPower(0);

    }





    // Odometry stuff



    public void buildPaths(){

        scorePreload = new Path(new BezierLine(startPose, scorePose));

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());



        grabPickup1a = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, pickupPose1))

                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose1.getHeading())

                .build();

        grabPickup1m = follower.pathBuilder()

                .addPath(new BezierLine(pickupPose1, pickupMidPose1))

                .setLinearHeadingInterpolation(pickupPose1.getHeading(), pickupMidPose1.getHeading())

                .build();

        grabPickup1m2 = follower.pathBuilder()

                .addPath(new BezierLine(pickupMidPose1, pickupMid2Pose1))

                .setLinearHeadingInterpolation(pickupMidPose1.getHeading(), pickupMid2Pose1.getHeading())

                .build();

        grabPickup1b = follower.pathBuilder()

                .addPath(new BezierLine(pickupMid2Pose1, pickupEndPose1))

                .setLinearHeadingInterpolation(pickupMid2Pose1.getHeading(), pickupEndPose1.getHeading())

                .build();

        scorePickup1 = follower.pathBuilder()

                .addPath(new BezierLine(pickupEndPose1, scorePose))

                .setLinearHeadingInterpolation(pickupEndPose1.getHeading(), scorePose.getHeading())

                .build();

        grabPickup2a = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, pickupPose2))

                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose2.getHeading())

                .build();

        grabPickup2m = follower.pathBuilder()

                .addPath(new BezierLine(pickupPose2, pickupMidPose2))

                .setLinearHeadingInterpolation(pickupPose2.getHeading(), pickupMidPose2.getHeading())

                .build();

        grabPickup2m2 = follower.pathBuilder()

                .addPath(new BezierLine(pickupMidPose2, pickupMid2Pose2))

                .setLinearHeadingInterpolation(pickupMidPose2.getHeading(), pickupMid2Pose2.getHeading())

                .build();

        grabPickup2b = follower.pathBuilder()

                .addPath(new BezierLine(pickupMid2Pose2, pickupEndPose2))

                .setLinearHeadingInterpolation(pickupMid2Pose2.getHeading(), pickupEndPose2.getHeading())

                .build();

        scorePickup2 = follower.pathBuilder()

                .addPath(new BezierLine(pickupEndPose2, scorePose))

                .setLinearHeadingInterpolation(pickupEndPose2.getHeading(), scorePose.getHeading())

                .build();

        grabPickup3a = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, pickupPose3))

                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose3.getHeading())

                .build();

        grabPickup3b = follower.pathBuilder()

                .addPath(new BezierLine(pickupPose3, pickupEndPose3))

                .setLinearHeadingInterpolation(pickupPose3.getHeading(), pickupEndPose3.getHeading())

                .build();

        scorePickup3 = follower.pathBuilder()

                .addPath(new BezierLine(pickupEndPose3, scorePose))

                .setLinearHeadingInterpolation(pickupEndPose3.getHeading(), scorePose.getHeading())

                .build();

        end = follower.pathBuilder()

                .addPath(new BezierLine(scorePose, endPose))

                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())

                .build();

    }



    public void autonomousPathUpdate() {

        switch (pathState) {



            case 0: // prep shoot + go to shooting position

                prepShoot();

                follower.followPath(scorePreload);

                setPathState(1);

                break;



            case 1: // make sure we're in position and prepped for enough time

                if (follower.isBusy() || pathTimer.getElapsedTimeSeconds() < prepShootTime) return;



                intake();

                shootTimer.resetTimer();



                setPathState(2);

                break;

            case 2: // shoot

                if (shootTimer.getElapsedTimeSeconds() < shootTime) {

                    shoot();

                } else {

                    // stop shooting and go pick up

                    stopShooting();

                    follower.followPath(grabPickup1a, true);



                    setPathState(3);

                }

                break;



            case 3: // pickup 1

                if (!follower.isBusy()) {

                    slowIntake();

                    follower.setMaxPower(speedMovePercent);

                    follower.followPath(grabPickup1m, true);



                    waitTimer.resetTimer();



                    setPathState(20);

                }

                break;



            case 20: // pickup 2

                if (follower.isBusy() || waitTimer.getElapsedTimeSeconds() < stabilizeTime) return;

                slowIntake();

                follower.setMaxPower(speedMovePercent);

                follower.followPath(grabPickup1m2, true);



                waitTimer.resetTimer();



                setPathState(22);

                break;



            case 22: // pickup 3

                // drive to pickup during intake

                if (follower.isBusy() || waitTimer.getElapsedTimeSeconds() < stabilizeTime) return;

                slowIntake();

                follower.setMaxPower(speedMovePercent);

                follower.followPath(grabPickup1b, true);

                waitTimer.resetTimer();



                setPathState(4);

                break;



            case 4: // finish intake (push balls back a little)

                if (!follower.isBusy()) {

                    reverseIntake();

                    waitTimer.resetTimer();



                    setPathState(24);

                }

                break;



            case 24: // move to scoring position

                if (!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.45) {

                    follower.setMaxPower(1.0);

                    // Spin up shooter BEFORE moving

                    stopShooting();

                    prepShoot();

                    follower.followPath(scorePickup1, true);

                    setPathState(5);

                }

                break;

            case 5: // make sure we're in position and prepped for enough time

                if (follower.isBusy() || pathTimer.getElapsedTimeSeconds() < prepShootTime) return;

                intake();

                shootTimer.resetTimer();

                setPathState(6);

                break;



            case 6: // shoot

                if (shootTimer.getElapsedTimeSeconds() < shootTime) {

                    shoot();

                } else {

                    // stop shooting and go pick up

                    stopShooting();

                    follower.followPath(grabPickup2a, true);



                    setPathState(7);

                }

                break;



            case 7: // pickup 1

                if (!follower.isBusy()) {

                    slowIntake();

                    follower.setMaxPower(speedMovePercent);

                    follower.followPath(grabPickup2m, true);



                    waitTimer.resetTimer();



                    setPathState(21);

                }

                break;



            case 21: // pickup 2

                if (follower.isBusy() || waitTimer.getElapsedTimeSeconds() < stabilizeTime) return;

                slowIntake();

                follower.setMaxPower(speedMovePercent);

                follower.followPath(grabPickup2m2, true);



                waitTimer.resetTimer();



                setPathState(23);

                break;

            case 23: // pickup 3

                if (follower.isBusy() || waitTimer.getElapsedTimeSeconds() < stabilizeTime) return;

                slowIntake();

                follower.setMaxPower(speedMovePercent);

                follower.followPath(grabPickup2b, true);



                setPathState(25);

                break;

            case 25: // finish intake (push balls back a little)

                if (!follower.isBusy()) {

                    reverseIntake();

                    waitTimer.resetTimer();



                    setPathState(8);

                }

                break;

            case 8: // move to scoring position

                if (!follower.isBusy()) {

                    follower.setMaxPower(1.0);

                    // Spin up shooter BEFORE moving

                    stopShooting();

                    prepShoot();

                    follower.followPath(scorePickup2, true);

                    setPathState(9);

                }

                break;



            case 9: // make sure we're in position and prepped for enough time

                if (follower.isBusy() || pathTimer.getElapsedTimeSeconds() < prepShootTime) return;

                intake();

                shootTimer.resetTimer();

                setPathState(10);

                break;



            case 10: // shoot

                if (shootTimer.getElapsedTimeSeconds() < shootTime) {

                    shoot();

                } else {

                    // stop shooting and go pick up

                    stopShooting();

                    follower.followPath(end, true);



                    setPathState(11);

                }

                break;



            case 11: // leave points

                if (!follower.isBusy()) {

                    stopShooting();

                    follower.followPath(end);

                    setPathState(-1);

                }

                break;

        }

    }



    public void setPathState(int newState) {

        pathState = newState;



        // Reset ALL timers when switching states

        pathTimer.resetTimer();

    }



    @Override

    public void init() {



        // Robot motors



//         leftFrontDrive = hardwareMap.get(DcMotor.class,"leftFrontDrive");

//         rightFrontDrive = hardwareMap.get(DcMotor.class,"rightFrontDrive");

//         leftRearDrive = hardwareMap.get(DcMotor.class,"leftRearDrive");

//         rightRearDrive = hardwareMap.get(DcMotor.class,"rightRearDrive");

        intakeServo = hardwareMap.get(CRServo.class,"EntryServo");

        intakeServo.setPower(0);

        leftConveyor = hardwareMap.get(DcMotor.class,"leftConveyor");

        rightConveyor = hardwareMap.get(DcMotor.class,"rightConveyor");

        leftLaunch = hardwareMap.get(DcMotor.class,"leftLaunch");

        rightLaunch = hardwareMap.get(DcMotor.class,"rightLaunch");



        rightLaunch.setDirection(DcMotor.Direction.FORWARD);

        leftLaunch.setDirection(DcMotor.Direction.REVERSE);

        rightConveyor.setDirection(DcMotor.Direction.FORWARD);

        leftConveyor.setDirection(DcMotor.Direction.REVERSE);



        myVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        double currVoltage = myVoltageSensor.getVoltage();



        shootConstant = -0.0337841*currVoltage+0.78081;



        // Odometry variables

        pathTimer = new Timer();

        opmodeTimer = new Timer();

        shootTimer = new Timer();

        waitTimer = new Timer();

        opmodeTimer.resetTimer();



        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);

    }





    @Override

    public void loop(){

        follower.update();

        autonomousPathUpdate();



        telemetry.addData("Path state",pathState);

        telemetry.addData("y",follower.getPose().getX());

        telemetry.addData("x",follower.getPose().getY());

        telemetry.addData("Heading",pathState);

        telemetry.update();

    }



    @Override

    public void init_loop() {}



    @Override

    public void start(){

        opmodeTimer.resetTimer();

        setPathState(0);

    }



    @Override

    public void stop() {}



}