package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class AndewFarDoubleHive extends MovmentMain {
    public Pose startPose;
    public Pose nav1;
    public Pose nav2;
    public Pose shootingPose;
    public Pose nav3;
    public Pose parkPose;

    public Path start_to_shoot;
    public Path shoot_to_park;

    public void buildPaths(){
        start_to_shoot = path(
                line(startPose, nav1).constant(startPose),
                line(nav1, nav2).tangent(),
                line(nav2, shootingPose).constant(shootingPose)
        );
        shoot_to_park = path(
                //line(shootingPose, nav3).tangent(),
                line(shootingPose, parkPose).constant(parkPose)
        );
    }

    @Override
    public void pathLogic(){
        if(moveToPointChain.equals(start_to_shoot)) {
            launch(930, 930, 3, 1);
            if (opmodeTimer.seconds() > 15) {
                cutPower();
                movePath(shoot_to_park, pathingState.PARK);
            }
        }
        else{
            launch(930, 930, 3, 1);
            if (opmodeTimer.seconds() > 6) {
                cutPower();
                movePath(start_to_shoot, pathingState.LOOP);
            }
        }
    }
}
