package org.firstinspires.ftc.teamcode.RobotFunctions.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This class contains the hardware and methods for the sensor subsystem
 * Any angle reported in degrees will have a range of 0-360
 * The imu is under i2c port 0 in the robot controller configuration
 */

public class Sensors {
    public BNO055IMU imu;
    HardwareMap map;

    public Sensors(HardwareMap map){
        this.map = map;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // calibration from sample calibration opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    Orientation angle;
    double heading;

    public double getHeading(){
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = (angle.firstAngle + 360) % 360; // converts 180 -- -180 range to 0 - 360
        return heading;
    }

    double degrees, radians;
    Orientation degree;

    public double getRadian(){
        degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        radians = Math.toRadians(((degree.firstAngle + 360) % 360)); // converts 180 -- -180 range to 0 - 360 then converts to radians
        return radians;
    }
}
