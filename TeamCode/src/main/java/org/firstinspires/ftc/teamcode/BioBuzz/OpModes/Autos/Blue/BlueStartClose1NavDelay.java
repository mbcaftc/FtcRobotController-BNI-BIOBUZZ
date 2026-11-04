package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Start_At_Hive_close", group = "Drive")
public class BlueStartClose1NavDelay extends CloseHiveTipDelay{
    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())
        startPose = StartCloseLeft;
        nav1 = Quadrant3Center;
        parkPose = ParkPoseB;
        startDelay = 10;
        shootTime = 8.0;
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }
}
