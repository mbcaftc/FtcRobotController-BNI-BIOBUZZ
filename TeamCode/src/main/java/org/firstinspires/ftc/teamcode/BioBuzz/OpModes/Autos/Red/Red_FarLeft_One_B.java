package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Red;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class Red_FarLeft_One_B extends MovmentMain {

    public Pose startPose;
    public Pose shootPose;
    public Pose transitPose;
    public Pose parkPose;

    public Path start_to_shoot;
    public Path shoot_to_transit;
    public Path transit_to_park;

    public void buildPaths() {
        start_to_shoot = createLinearPath(startPose, shootPose, 270, 270);
        shoot_to_transit = createLinearPath(shootPose, transitPose, 270, 270);
        transit_to_park = createLinearPath(transitPose, parkPose, 270, 90);
    }

    @Override
    public void pathLogic() {
        //If done with the shooting path, go to the parking position and stop the loop by handing off to the park state.
        if (moveToPointChain.equals(start_to_shoot)) {
            launch(1000, 1);
            if (opmodeTimer.seconds() > 10) {
                movePath(shoot_to_transit, pathingState.LOOP);
            }
        } else if (moveToPointChain.equals(shoot_to_transit)){
            //If there is not path assigned to moveToPointChain, assign it the first path of start_to_shoot
            movePath(transit_to_park, pathingState.PARK);
        }else{
            ;
//            //If there is not path assigned to moveToPointChain, assign it the first path of start_to_shoot
//            movePath(start_to_shoot, MovmentMain.pathingState.LOOP);
        }
    }
    @Override
    public void init(){
        //AutoMain init
        initHardWare();
        setup();
        //Set BlueMain points (BEFORE buildPaths())
        startPose = RedStartFarLeft;
        shootPose = RedQuadrant1Center;
        parkPose = RedParkPoseB;
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }
}