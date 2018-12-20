package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    HardwareMap map;
    LinearOpMode linOpMode;

    public DcMotorEx pivot, flaps;

    public Intake(HardwareMap hmap, LinearOpMode linOpMode){
        map = hmap;
        this.linOpMode = linOpMode;
        pivot = (DcMotorEx) map.dcMotor.get("pivot");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flaps = (DcMotorEx) map.dcMotor.get("flaps");
    }

    public Intake(HardwareMap hmap){
        map = hmap;
        pivot = (DcMotorEx) map.dcMotor.get("pivot");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flaps = (DcMotorEx) map.dcMotor.get("flaps");
    }
}
