package org.firstinspires.ftc.teamcode.RobotFunctions.TheAllSeeingRobot;

import android.app.Activity;
import android.content.Context;
import android.view.Surface;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * This class provides a base class for interfacing opencv with opencv detectors
 *
 * @see github.com/guineawheek/endercv/blob/master/endercv/src/main/java/org/corningrobotics/enderbots/endercv/OpenCVPipeline.java for the original source of this code
 *
 * TODO: remove the stopped bool function, replace in detectors with checking opmode
 */

public abstract class OpenCVpipeline implements CameraBridgeViewBase.CvCameraViewListener2{
    static {
        System.loadLibrary("opencv_java3"); //idk what this does but without it the rc crashes
    }

    public LinearOpMode opmode;

    private Context context;
    private ViewDisplay viewDisplay;
    private boolean inited;
    private JavaCameraView cameraView;

    public void init(Context context, ViewDisplay viewDisplay, final int cameraIndex, LinearOpMode opmode){
        this.opmode = opmode;
        this.context = context;
        this.viewDisplay = viewDisplay;
        final Context finalContext = context;
        final Activity activity = (Activity) context;
        final CameraBridgeViewBase.CvCameraViewListener2 self = this;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                // JCVs must be instantiated on a UI thread
                cameraView = new JavaCameraView(finalContext, cameraIndex);
                cameraView.setCameraIndex(cameraIndex);
                cameraView.setCvCameraViewListener(self);
                inited = true;
            }
        });
    }

    public void init(Context context, ViewDisplay viewDisplay, final int cameraIndex){
        this.context = context;
        this.viewDisplay = viewDisplay;
        final Context finalContext = context;
        final Activity activity = (Activity) context;
        final CameraBridgeViewBase.CvCameraViewListener2 self = this;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                // JCVs must be instantiated on a UI thread
                cameraView = new JavaCameraView(finalContext, cameraIndex);
                cameraView.setCameraIndex(cameraIndex);
                cameraView.setCvCameraViewListener(self);
                inited = true;
            }
        });
    }

    public void enable(){
        cameraView.enableView();
        viewDisplay.setCurrentView(context, getCameraView());
    }

    public JavaCameraView getCameraView() {
        return cameraView;
    }

    public void disable() {
        cameraView.disableView();
        viewDisplay.removeCurrentView(context);
    }


    public void onCameraViewStarted(int width, int height) { // opencv needs it cause idk
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) { // I think it returns a mat with a camera frame input, idk
        Mat rgba = new Mat();
        Mat gray = new Mat();

        switch (((Activity) context).getWindowManager().getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_0:
                // this breaks horribly for some reason
                Core.rotate(inputFrame.rgba(), rgba, Core.ROTATE_90_CLOCKWISE);
                Core.rotate(inputFrame.gray(), gray, Core.ROTATE_90_CLOCKWISE);
                break;
            case Surface.ROTATION_90:
                rgba = inputFrame.rgba();
                gray = inputFrame.gray();
                break;
            case Surface.ROTATION_270:
                Core.rotate(inputFrame.rgba(), rgba, Core.ROTATE_180);
                Core.rotate(inputFrame.gray(), gray, Core.ROTATE_180);
                break;
        }
        return processFrame(rgba, gray);
    }

    public abstract Mat processFrame(Mat rgba, Mat gray); // is implemented in detector class and does the processing for the image

    @Override
    public void onCameraViewStopped() {// override this and add code if you need stuff to happen when camera stopped
    }


}
