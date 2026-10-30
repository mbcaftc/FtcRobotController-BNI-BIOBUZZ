package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class CloseHiveTipDelay extends MovmentMain {
    public Pose startPose;
    public Pose nav1;
    public Pose parkPose;

    public Path start_to_park;
    public double startDelay;
    public Double shootTime;




    public void buildPaths(){
        Pose p0 = new Pose(startPose.x(), startPose.y());
        Pose p1 = new Pose(nav1.x(), nav1.y());
        Pose p2 = new Pose(parkPose.x(), parkPose.y());

        double startAngle = Math.toRadians(90);
        double endAngle = Math.toRadians(180);

        start_to_park = path(line(p0, p1), line(p1, p2)).linear(startAngle, endAngle);
    }




    @Override
    public void pathLogic(){
        if (shootTime == null) {
            shootTime = 8.0;
        }
        double currentTime = opmodeTimer.seconds();

        if (currentTime < startDelay) {}
        else if (currentTime >= startDelay && currentTime < startDelay + shootTime) {
            launch(960, 1225, 3, 1);
        }
        else {
            cutPower();
            movePath(start_to_park, pathingState.PARK);
        }

    }
}
