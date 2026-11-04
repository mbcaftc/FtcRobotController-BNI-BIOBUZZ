package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class AndewFarHiveTip extends MovmentMain {
    public AndrewPose startPose;
    public AndrewPose nav1;
    public AndrewPose nav2;
    public AndrewPose parkPose;

    public Path start_to_park;

    public void buildPaths(){

        Pose p0 = startPose.poseHeadingDegrees(270);
        Pose p1 = nav1.poseHeadingDegrees(180);
        Pose p2 = nav2.poseHeadingDegrees(90);
        Pose p3 = parkPose.poseHeadingDegrees(0);


        start_to_park = path(
                line(p0, p1),
                line(p1, p2),
                line(p2, p3)
        ).tangent();
    }

    @Override
    public void pathLogic(){
        launch(960, 1225, 3, 1);
        if (opmodeTimer.seconds() > 10) {
            cutPower();
            movePath(start_to_park, pathingState.PARK);
        }
    }
}
