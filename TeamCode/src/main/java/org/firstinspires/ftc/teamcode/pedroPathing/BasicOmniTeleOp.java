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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class BasicOmniTeleOp extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotorEx flywheel;
    private Servo kicker;

    private DcMotorEx intake;

    private boolean isRotatingToTarget = false;
    private double targetHeading = 0;
    private boolean rightStickPressed = false;
    private boolean leftStickPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware
        flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kicker = hardwareMap.get(Servo.class, "kicker");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Set zero power behaviour of the flywheels
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Tune PIDF for flywheel
        flywheel.setVelocityPIDFCoefficients(5.5, 0, 0, 15);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // Joystick Movement Variables
        double line = -gamepad1.left_stick_y*0.6;
        double strafe = -gamepad1.left_stick_x*0.6;
        double turn = -gamepad1.right_stick_x * 0.4;

        // Speed of Micro Adjustments
        double microSpeed = 0.1; // Adjust this (lower = slower)

        // Quick Rotation Angle
        double quickRotationAngle = 180.0;

        // Micro Movement Control
        if (gamepad1.dpad_up) {
            line = microSpeed;
            strafe = 0.0;
        } else if (gamepad1.dpad_down) {
            line = -microSpeed;
            strafe = 0.0;
        } else if (gamepad1.dpad_right) {
            line = 0.0;
            strafe = -microSpeed;
        } else if (gamepad1.dpad_left) {
            line = 0.0;
            strafe = microSpeed;
        }

        // Micro Rotation Control
        if (gamepad1.right_bumper) {
            turn = -microSpeed;
        } else if (gamepad1.left_bumper) {
            turn = microSpeed;
        }

        // Quick Rotation Control
        if (gamepad1.right_stick_button && !rightStickPressed && !isRotatingToTarget) {
            rightStickPressed = true;
            double currentHeading = Math.toDegrees(follower.getPose().getHeading());
            targetHeading = Math.toRadians(currentHeading - quickRotationAngle);
            isRotatingToTarget = true;
        } else if (!gamepad1.right_stick_button) {
            rightStickPressed = false;
        }

        // If rotating to target, override turn control
        if (isRotatingToTarget) {
            double currentHeading = follower.getPose().getHeading();
            double headingError = targetHeading - currentHeading;

            // Normalize error to -PI to PI
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Stop if close enough (within 1 degree)
            if (Math.abs(Math.toDegrees(headingError)) < 1.0) {
                turn = 0;
                isRotatingToTarget = false;
            } else {
                // Proportional control - turn towards target
                turn = headingError * 0.5; // Adjust multiplier for speed
            }
        }

        // Set gamepad controls
        follower.setTeleOpDrive(line, strafe, turn, true);

        // Small Flywheel Control
        if (gamepad1.y) {
            kicker.setPosition(0);
        } else {
            kicker.setPosition(0.90);
        }

        // Flywheel control
        if (gamepad1.left_trigger > 0.1) {
            rotateFlywheel(1100);
        } else {
            rotateFlywheel(0.0);
        }

        if(gamepad1.right_trigger > 0.1){
            intake.setPower(1);
        }

        telemetryUpdate();
    }

    private void telemetryUpdate() {
        // Controls Manual
        telemetry.addLine("====CONTROLS====");
        telemetry.addLine("Left Joystick: Movement");
        telemetry.addLine("Right Joystick: Rotation");
        telemetry.addLine("Right Joystick Button: Rotate 180 degrees clockwise");
        telemetry.addLine("Left Trigger: Flywheel");
        telemetry.addLine("D-Pad: Microadjustments for movement");
        telemetry.addLine("Left + Right Bumper: Microadjustments for rotation");

        // Info
        telemetry.addLine("\n====ROBOT INFO====");
        telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("launcher velocity", flywheel.getVelocity());

        telemetry.update();
    }

    private void rotateFlywheel(double power) {
        flywheel.setVelocity(power);
    }

}