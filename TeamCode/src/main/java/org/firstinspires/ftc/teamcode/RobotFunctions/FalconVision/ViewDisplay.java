package org.firstinspires.ftc.teamcode.RobotFunctions.FalconVision;

import android.content.Context;
import android.view.View;

/**
 * I think this class adds methods to add images onto the robot controller screen, not too sure
 * Just don't touch anything in this interface and it'll be fine
 *
 * @author ethan
 */

public interface ViewDisplay { //adds methods to add image to screen
    void setCurrentView(Context context, View view);
    void removeCurrentView(Context context);
}
