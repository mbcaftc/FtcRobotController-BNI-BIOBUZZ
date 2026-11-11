package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class ShootFarUnderHiveBase extends MovmentMain {
    public Pose startPose;
    public Pose nav1;
    public Pose nav2;
    public Pose parkPose;

    public Path start_to_park;

    public void buildPaths(){
        start_to_park = path(
                line(startPose, nav1).constant(startPose),
                line(nav1, nav2).constant(startPose),
                line(nav2, parkPose).constant(startPose)
        );
    }

    @Override
    public void pathLogic(){
        if (opmodeTimer.seconds() > 17) {
            cutPower();
            movePath(start_to_park, pathingState.PARK);
        }
        else{
            launch(970, 980, 3, 1);
        }
    }
}
