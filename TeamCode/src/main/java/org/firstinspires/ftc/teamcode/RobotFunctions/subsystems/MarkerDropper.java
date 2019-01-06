package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MarkerDropper {
    HardwareMap map;
    LinearOpMode linOpMode;

    public Servo dropper;

    public MarkerDropper(HardwareMap hmap, LinearOpMode linOpMode){
        map = hmap;
        this.linOpMode = linOpMode;
        dropper = map.servo.get("dropper");
    }

    public MarkerDropper(HardwareMap hmap){
        map = hmap;
        dropper = map.servo.get("dropper");
    }
}
