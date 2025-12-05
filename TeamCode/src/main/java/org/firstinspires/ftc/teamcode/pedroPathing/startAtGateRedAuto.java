package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "startAtGateRedAuto", group = "Pedro Pathing")
public class startAtGateRedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    // Start Pose
    private final Pose startPose = new Pose(122, 122, Math.toRadians(220));

    private final Pose moveBackPose = new Pose(112,112,Math.toRadians(220));
    // End Pose
    private final Pose endPose = new Pose(90, 140, Math.toRadians(270));

    private Path forwardPath;

    private Path toEndPose;

    private Path stayPath;

    private DcMotorEx flywheel;
    private CRServo leftServo, rightServo;


    public void buildPaths() {
        /* Simple forward path using BezierLine (straight line) */
        forwardPath = new Path(new BezierLine(startPose, moveBackPose));
        forwardPath.setLinearHeadingInterpolation(startPose.getHeading(), moveBackPose.getHeading());

        toEndPose = new Path(new BezierLine(moveBackPose, endPose));
        toEndPose.setLinearHeadingInterpolation(moveBackPose.getHeading(), endPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the forward path
                boolean shot = shooter();
                if(shot == true){
                    setPathState(1);
                }
                break;
            case 1:
                // Wait until path is complete
                if (!follower.isBusy()) {
                    follower.followPath(forwardPath);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(toEndPose);
                    setPathState(-1);
                }
                break;
        }
    }

    private boolean shooter (){
        Timer shootingTimer = new Timer();
        long timeAtShooting = shootingTimer.getElapsedTime();
        while(shootingTimer.getElapsedTime() - timeAtShooting< 11000){
            rotateFlywheel(1515.0);
            if (shootingTimer.getElapsedTime() - timeAtShooting > 5000 &&
                    shootingTimer.getElapsedTime() - timeAtShooting<11000){
                rotateServos(0.45);
            }
        }
        //slowing down flywheel
        while(shootingTimer.getElapsedTime()>11000 &&
        shootingTimer.getElapsedTime() < 12000){
            rotateFlywheel(-10);
        }
        rotateFlywheel(0.0);
        rotateServos(0.0);
        return true;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addData("Path timer", pathTimer.getElapsedTime());
        telemetry.addData("flywheel velocty", flywheel.getVelocity());

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hardware
        flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftServo = hardwareMap.get(CRServo.class, "leftFeeder");
        rightServo = hardwareMap.get(CRServo.class, "rightFeeder");

        // Reverse as necessary
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behaviour of the flywheels
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Tune PIDF for flywheel
        flywheel.setVelocityPIDFCoefficients(4.3, 0, 0, 9);
    }

    private void rotateFlywheel(double power) {
        flywheel.setVelocity(power);
    }

    private void rotateServos(double power) {
        rightServo.setPower(power);
        leftServo.setPower(power);
    }

    @Override
    public void init_loop() {
        // In the future have starting positions to choose
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}