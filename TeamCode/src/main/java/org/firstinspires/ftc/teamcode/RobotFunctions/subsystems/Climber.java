package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    HardwareMap map;
    LinearOpMode linOpMode;

    public DcMotorEx climb; //positive power moves climber counterclockwise
    public Climber(HardwareMap hmap, LinearOpMode linOpMode){
        map = hmap;
        this.linOpMode = linOpMode;
        climb = (DcMotorEx) map.dcMotor.get("climb");
    }

    public Climber(HardwareMap hmap){
        map = hmap;
        climb = (DcMotorEx) map.dcMotor.get("climb");
    }
}
