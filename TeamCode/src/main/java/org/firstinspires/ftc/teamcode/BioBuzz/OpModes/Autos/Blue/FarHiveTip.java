package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class FarHiveTip extends MovmentMain {
    public Pose startPose;
    public Pose nav1;
    public Pose nav2;
    public Pose parkPose;

    public Path start_to_park;

    public void buildPaths(){

        Pose p0 = new Pose(startPose.x(), startPose.y());
        Pose p1 = new Pose(nav1.x(), nav1.y());
        Pose p2 = new Pose(nav2.x(), nav2.y());
        Pose p3 = new Pose(parkPose.x(), parkPose.y());


        start_to_park = path(
                line(p0, p1),
                line(p1, p2),
                line(p2, p3)
        ).constant(Math.toRadians(270));
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
