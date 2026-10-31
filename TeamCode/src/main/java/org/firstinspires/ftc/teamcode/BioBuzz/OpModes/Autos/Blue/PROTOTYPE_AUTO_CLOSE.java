package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Reinstated Blue_CloseLeft_One_A as of 16f8b21 (lerp shooting). Move to shoot, shoot, move straight to park.
@Autonomous(name = "PROTOTYPE_AUTO_CLOSE", group = "Drive")
public class PROTOTYPE_AUTO_CLOSE extends BlueMain{
    //Seconds to wait after start before moving
    public double startDelay = 10;

    //Same as BlueMain.pathLogic(), shifted later by startDelay
    @Override
    public void pathLogic(){
        //If done with the shooting path, go to the parking position and stop the loop by handing off to the park state.
        if (moveToPointChain.equals(start_to_shoot)) {
            launch(1050, 1500, 3, 1);
            if (opmodeTimer.seconds() > startDelay + 10) {
                cutPower();
                movePath(shoot_to_park, pathingState.PARK);
            }
        }
        else if (opmodeTimer.seconds() >= startDelay){
            //After the delay, start the first path of start_to_shoot
            movePath(start_to_shoot, pathingState.LOOP);
            //Spin up launcher early
            launcherMotor.setVelocity(1100);
        }
    }

    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildPaths())
        startPose = StartCloseLeft;
        shootPose = Quadrant3Back;
        parkPose = ParkPoseB;
        //BlueMain init
        buildPaths();
        //Start the follower
        follower.setPose(new Pose(startPose.x(), startPose.y(), Math.toRadians(90)));
        follower.update();
    }
}
