package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class AndewFarHiveTip extends MovmentMain {
    public Pose startPose;
    public Pose nav1;
    public Pose nav2;
    public Pose parkPose;

    public Path start_to_park;

    public void buildPaths(){
        start_to_park = path(
                line(startPose, nav1).constant(startPose),
                line(nav1, nav2).tangent(),
                line(nav2, parkPose).constant(parkPose)
        );
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
