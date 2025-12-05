package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "RedfrontAuto", group = "Pedro Pathing")
public class redFrontAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    // Start Pose
    private final Pose startPose = new Pose(84, 12, Math.toRadians(270));

    private final Pose intermPose = new Pose(84,72,Math.toRadians(270));
    // End Pose

    private Path forwardPath;


    private DcMotorEx flywheel;
    private CRServo leftServo, rightServo;


    public void buildPaths() {
        /* Simple forward path using BezierLine (straight line) */
        forwardPath = new Path(new BezierLine(startPose, intermPose));
        forwardPath.setLinearHeadingInterpolation(startPose.getHeading(), intermPose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the forward path
                follower.followPath(forwardPath);
                setPathState(1);
                break;
            case 1:
                // Wait until path is complete
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
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
        //flywheel.setVelocityPIDFCoefficients(0, 0, 0, 0);
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