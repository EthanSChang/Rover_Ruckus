package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

//this class allows a gamepad to control the climber, for testing encoder positions
@TeleOp
public class ClimbTest extends OpMode {
    TankHardware robot = new TankHardware();
    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
        robot.climber.climb.setPower(gamepad1.left_stick_y);
        telemetry.addData("motor power", robot.climber.climb.getPower());
        telemetry.addData("motor encoder position", robot.climber.climb.getCurrentPosition());
        telemetry.update();
    }
}
