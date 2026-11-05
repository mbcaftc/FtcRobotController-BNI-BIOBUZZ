package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue_NearCenter_One_A", group = "Drive")
public class Blue_CloseLeft_One_A extends BlueMain{
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())
        startPose = StartCloseLeft.pose();
        shootPose = Quadrant3Back.pose();
        parkPose = ParkPoseB.pose();
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }
}
