package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class BasicOmniTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private DcMotorEx flywheel;

    private CRServo leftFeeder;

    private CRServo rightFeeder;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        // Intiailize the flywheel
        flywheel = hardwareMap.get(DcMotorEx.class,"launcher");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();

        // When Y is pressed
        if (gamepad1.y) {
            if (flywheel.getVelocity() == 0) { // If the flywheel is off, turn it on
                flywheel.setVelocity(1000);
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
            } else {
                flywheel.setVelocity(0); // If the flywheel is on, turn it off
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            }
        }

        // These set joystick movement and rotational controls
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y, // Move left and right
                -gamepad1.left_stick_x, // Move forward and backward
                -gamepad1.right_stick_x, // Rotate left and right
                true
        );
    }
}