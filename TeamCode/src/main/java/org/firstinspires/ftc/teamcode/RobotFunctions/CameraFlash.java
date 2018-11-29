package org.firstinspires.ftc.teamcode.RobotFunctions;

import android.hardware.Camera;

import static android.hardware.Camera.Parameters.FLASH_MODE_OFF;
import static android.hardware.Camera.Parameters.FLASH_MODE_TORCH;

public class CameraFlash {

    private Camera camera;
    private Camera.Parameters parm;

    public void on(){
        camera = Camera.open();
        parm = camera.getParameters();
        parm.setFlashMode(FLASH_MODE_TORCH);
        camera.setParameters(parm);
    }

    public void off(){
        parm.setFlashMode(FLASH_MODE_OFF);
        camera.setParameters(parm);
        camera.release();
    }
}