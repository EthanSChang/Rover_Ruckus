package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

@TeleOp
@Disabled
public class LimitSwitchTest extends OpMode {
    TankHardware robot = new TankHardware();

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
        telemetry.addData("low switch", robot.climber.limLow.getState());
        telemetry.addData("high switch", robot.climber.limHigh.getState());
        telemetry.update();
    }
}
