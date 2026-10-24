package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue_CloseLeft_One_A", group = "Drive")
public class Blue_CloseLeft_One_A extends BlueMain{
    @Override
    public void init(){
        //AutoMain init
        setup();
        //Set BlueMain points (BEFORE buildPaths())
        startPose = StartCloseLeft;
        shootPose = Quadrant3Center;
        parkPose = ParkPoseB;
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), 90));
        follower.update();
    }
}
