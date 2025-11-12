package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="Teleop30", group="TeleOp")

public class Teleop30645 extends OpMode {
    DriveSetup30645 drive = new DriveSetup30645();
    Launcher launcher = new Launcher();
    double forward, strafe, rotate;

    @Override
    public void init(){
        drive.init(hardwareMap);
        launcher.init(hardwareMap);
    }


    public void loop(){
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldOriented(forward, strafe, rotate);

        if(gamepad1.y){
            launcher.startLauncher();
        }
        else if(gamepad1.b){
            launcher.stopLauncher();
        }

        launcher.updateState();

        telemetry.addData("State", launcher.getState());
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
    }
}
