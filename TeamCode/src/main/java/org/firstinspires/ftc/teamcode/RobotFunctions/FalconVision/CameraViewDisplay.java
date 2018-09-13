package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision;

import android.app.Activity;
import android.content.Context;
import android.view.View;
import android.view.ViewGroup;

/**
 * This class adds the capability to add a live feed of the camera from the robot controller, to the screen on the robot controller
 * I'm not sure exactly how it works. Just don't touch anything and it'll be fine
 *
 * @author ethan
 */

public class CameraViewDisplay implements ViewDisplay {// adds camera image onto screen, idk how it works
    private static CameraViewDisplay instance;

    View view;

    public static CameraViewDisplay getInstance() {
        if (instance == null) instance = new CameraViewDisplay();
        return instance;
    }

    public void setCurrentView(Context context, View newView) {
        final int resID = context.getResources().getIdentifier("RelativeLayout", "id", context.getPackageName());
        final Activity activity = (Activity) context;
        final View queuedView = newView;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                ViewGroup l = (ViewGroup) activity.findViewById(resID); //R.id.RelativeLayout);
                if (view != null) {
                    l.removeView(view);
                }
                l.addView(queuedView);
                view = queuedView;
            }
        });
    }

    public void removeCurrentView(Context context) {
        final int resID = context.getResources().getIdentifier("RelativeLayout", "id", context.getPackageName());
        final Activity activity = (Activity) context;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //cameraMonitorViewId
                ViewGroup l = (ViewGroup) activity.findViewById(resID); // .id.RelativeLayout);
                if (view != null) {
                    l.removeView(view);
                }
                view = null;
            }
        });
    }
}
