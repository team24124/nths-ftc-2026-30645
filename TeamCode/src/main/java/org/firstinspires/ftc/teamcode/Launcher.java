package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    private final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    private final double FULL_SPEED = 1.0;
    private final double LAUNCHER_TARGET_VELOCITY = 1125;
    private final double LAUNCHER_MIN_VELOCITY = 1075;

    private DcMotorEx launcher;

    private CRServo leftFeeder, rightFeeder;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    LaunchState launchState;

    public void init(HardwareMap hwMap){
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hwMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hwMap.get(CRServo.class, "rightFeeder");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launchState = LaunchState.IDLE;
        stopFeeder();
        stopLauncher();
    }

    public void stopFeeder(){
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    public void updateState(){
        switch(launchState){
            case IDLE:
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if(launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY){
                    //change states
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                //transition
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if(feederTimer.seconds() > FEED_TIME_SECONDS){
                    //transition event
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public void startLauncher(){
        if(launchState == LaunchState.IDLE){
            //transition state
            launchState = LaunchState.SPIN_UP;
        }
    }

    public void stopLauncher(){
        stopFeeder();
        launcher.setVelocity(STOP_SPEED);
        launchState = LaunchState.IDLE;
    }

    public String getState(){
        return launchState.toString();
    }

    public double getVelocity(){
        return launcher.getVelocity();
    }

}
