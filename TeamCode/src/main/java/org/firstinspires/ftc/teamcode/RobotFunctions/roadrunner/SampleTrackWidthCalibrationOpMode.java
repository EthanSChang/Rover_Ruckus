package org.firstinspires.ftc.teamcode.RobotFunctions.roadrunner;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SampleTrackWidthCalibrationOpMode extends TrackWidthCalibrationOpMode {
    @Override
    protected Drive initDrive() {
        return new SampleTankDrive(hardwareMap);
    }

    @Override
    protected BNO055IMU initIMU() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        return imu;
    }
}