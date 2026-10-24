package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import static com.pedropathing.api.Paths.line;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;

public class AutoMain extends OpMode {
    public Timer pathTimer;
    public Timer opmodeTimer;
    public Follower follower;
    @Override
    public void init() {
        setup();
    }
    public void setup(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = MainConstants.createFollower(hardwareMap);
    }

    public void loop() {}

    public static Path createLinearPath(Pose start, Pose end, double headingStart, double headingEnd) {
        Pose a = new Pose(start.x(), start.y(), headingStart);
        Pose b = new Pose(end.x(), end.y(), headingEnd);
        return line(a, b).linear(a, b);
    }




}
