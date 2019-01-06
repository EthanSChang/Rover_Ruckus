package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.subsystems.Climber;

//this class allows a gamepad to control the climber, for testing encoder positions
@TeleOp
public class ClimbTest extends LinearOpMode {

    public void runOpMode(){
        Climber robot = new Climber(hardwareMap, this);
        waitForStart();
        while(opModeIsActive()){
            robot.climb.setPower(gamepad1.left_stick_y);
            telemetry.addData("motor power", robot.climb.getPower());
            telemetry.addData("motor encoder position", robot.climb.getCurrentPosition());
            telemetry.update();
        }

    }
}
