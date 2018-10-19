package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    HardwareMap map;
    LinearOpMode linOpMode;

    public DcMotorEx climb; //positive power moves climber counterclockwise
    public Climber(HardwareMap map, LinearOpMode linOpMode){
        this.map = map;
        this.linOpMode = linOpMode;
        climb = (DcMotorEx) map.dcMotor.get("climb");
    }

    public Climber(HardwareMap hmap){
        this.map = map;
        climb = (DcMotorEx) map.dcMotor.get("climb");
    }
}
