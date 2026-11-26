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


@Autonomous(name = "farredauton", group = "auton")
public class farredauton extends OpMode {

    // Shoot power!!
    private final double shootConstant = 0.35;

    private final double speedMovePercent = 0.7;


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
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(114, 135, Math.toRadians(90));
    private final Pose scorePose = new Pose(96,96,Math.toRadians(45));
    private final Pose pickupPose1 = new Pose(96,36,Math.toRadians(180));
    private final Pose pickupEndPose1 = new Pose(132,36,Math.toRadians(180));
    private final Pose endPose = new Pose(108,84,Math.toRadians(90));

    private Path scorePreload;
    private PathChain grabPickup1a, grabPickup1b, scorePickup1, grabPickup2a, grabPickup2b, scorePickup2, grabPickup3a, grabPickup3b, scorePickup3, end;


    // Shooting stuff

    private ElapsedTime pickupTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();

    private final double pickupTime = 4.0;

    private final double shootTime = 4.5;


    public void prepShoot(double shootPower) {
        leftLaunch.setPower(shootPower);
        rightLaunch.setPower(shootPower);
    }
    public void intake() {
        pickupTimer.reset();

        intakeServo.setPower(1.0);
        leftConveyor.setPower(0.3);
        rightConveyor.setPower(0.3);
    }

    public void intakeStop() {
        while (true) {
            if (pickupTimer.seconds() > pickupTime) {
                telemetry.addData("PickupTimer: ","At "+pickupTimer.seconds()+" out of "+pickupTime);
                telemetry.update();
                break;
            }
        }
//        pickupTimer.reset();

        intakeServo.setPower(0);
        leftConveyor.setPower(0);
        rightConveyor.setPower(0);
    }

    public void intakeReturn() {
        leftConveyor.setPower(-0.4);
        rightConveyor.setPower(-0.4);
        sleepTimer.reset();
        while (true) {
            if (sleepTimer.seconds() > 0.8) {
                break;
            }
        }
//        pickupTimer.reset();

        leftConveyor.setPower(0);
        rightConveyor.setPower(0);
    }

    public void stopShooting() {
        intakeServo.setPower(0);
        leftConveyor.setPower(0);
        rightConveyor.setPower(0);
        leftLaunch.setPower(0);
        rightLaunch.setPower(0);
    }

    public void wait(double time) {
        sleepTimer.reset();
        while (true) {
            if (sleepTimer.seconds() > time) {
                break;
            }
        }
    }




    // Odometry stuff

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        grabPickup1a = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose1.getHeading())
                .build();
        grabPickup1b = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose1, pickupEndPose1))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), pickupEndPose1.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupEndPose1, scorePose))
                .setLinearHeadingInterpolation(pickupEndPose1.getHeading(), scorePose.getHeading())
                .build();
        end = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                prepShoot(shootConstant);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    wait(0.5);
                    intake();
                    wait(4.0);
                    stopShooting();
                    follower.followPath(grabPickup1a,true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake();
                    follower.setMaxPower(speedMovePercent);
                    follower.followPath(grabPickup1b,true);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    intakeStop();
                    intakeReturn();
                    follower.setMaxPower(1.0);
                    prepShoot(shootConstant);
                    setPathState(4);
                    follower.followPath(scorePickup1, true);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    wait(1.2);
                    intake();
                    wait(shootTime);
                    stopShooting();
                    follower.followPath(grabPickup2a, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    intake();
                    follower.setMaxPower(speedMovePercent);
                    follower.followPath(grabPickup2b, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intakeStop();
                    intakeReturn();
                    follower.setMaxPower(1.0);
                    prepShoot(shootConstant);
                    follower.followPath(scorePickup2, true);
                    setPathState(10);
                }
                break;
//             case 7:
//                 if (!follower.isBusy()) {
//                     wait(0.5);
//                     intake();
//                     wait(shootTime);
//                     stopShooting();
//                     follower.followPath(grabPickup3a, true);
//                     setPathState(8);
//                 }
//                 break;
//             case 8:
//                 if (!follower.isBusy()) {
//                     intake();
//                     follower.setMaxPower(speedMovePercent);
//                     follower.followPath(grabPickup3b, true);
//                     setPathState(9);
//                 }
//                 break;
//             case 9:
//                 if (!follower.isBusy()) {
//                     intakeStop();
//                     follower.setMaxPower(1.0);
//                     prepShoot(shootConstant);
//                     follower.followPath(scorePickup3, true);
//                     setPathState(10);
//                 }
//                 break;
            case 10:
                if (!follower.isBusy()) {
                    wait(0.5);
                    intake();
                    wait(2.5);
                    stopShooting();
                    follower.followPath(end);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
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



        // Odometry variables

        pathTimer = new Timer();
        opmodeTimer = new Timer();
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
