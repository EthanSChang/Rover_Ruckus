package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.TankHardware;

import static org.firstinspires.ftc.teamcode.RobotFunctions.DashboardConstants.turnSpeed;

@TeleOp
public class ArcadeDrive extends OpMode {
    TankHardware robot = new TankHardware();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void init(){
        robot.init(hardwareMap);
        robot.driveTrain.SetBrake();
    }

    public void loop(){
        robot.driveTrain.arcadeDrive(gamepad1.left_stick_x * turnSpeed, gamepad1.left_stick_y);
        packet.put("back left pow", robot.driveTrain.bl.getPower());
        packet.put("back right pow", robot.driveTrain.br.getPower());
        dashboard.sendTelemetryPacket(packet);
    }
}
