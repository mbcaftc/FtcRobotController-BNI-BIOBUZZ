package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Red;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.BlueMain;

@Autonomous(name = "Red_CloseLeft_One_A", group = "Drive")
public class Red_CloseLeft_One_A extends RedMain {
    @Override
    public void init(){
        //AutoMain init
        setup();
        //Set BlueMain points (BEFORE buildPaths())
        startPose = RedStartCloseLeft;
        shootPose = RedQuadrant3Center;
        parkPose = RedParkPoseB;
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }
}
