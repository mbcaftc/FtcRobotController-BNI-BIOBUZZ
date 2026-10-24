package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

public class BlueMain extends MovmentMain {
    public Pose startPose;
    public Pose shootPose;
    public Pose parkPose;

    public Path start_to_shoot;
    public Path shoot_to_park;

    public void buildPaths(){
        start_to_shoot = createLinearPath(startPose, shootPose, 90, 90);
        shoot_to_park = createLinearPath(shootPose, parkPose, 90, 90);
    }

    @Override
    public void pathLogic(){
        //If done with the shooting path, go to the parking position and stop the loop by handing off to the park state.
        if (moveToPointChain.equals(start_to_shoot)) {
            movePath(shoot_to_park, pathingState.PARK);
        }
        else{
            //If there is not path assigned to moveToPointChain, assign it the first path of start_to_shoot
            movePath(start_to_shoot, pathingState.LOOP);
        }
    }
    private enum AutoState {
        START_TO_SHOOT,
        SHOOTING,
        SHOOT_TO_PARK,
        PARKED
    }
    private AutoState currentState;
    @Override
    public void pathLogic(boolean a) {
        //If done with the shooting path, go to the parking position and stop the loop by handing off to the park state.

        if (currentState == null){
            currentState = AutoState.START_TO_SHOOT;
        }
        switch (currentState){
            case START_TO_SHOOT:
                if (!follower.isBusy()) {
                    currentState = AutoState.SHOOTING;
                }
                break;
            case SHOOTING:
                launch(1000, 1);
                if (opmodeTimer.seconds() > 10) {
                    currentState = AutoState.SHOOT_TO_PARK;
                }
                break;
            case SHOOT_TO_PARK:
                if (!moveToPointChain.equals(shoot_to_park)) {
                    movePath(shoot_to_park, pathingState.PARK);
                }

                if (!follower.isBusy()) {
                    currentState = AutoState.PARKED;
                }
                break;
            case PARKED:
                break;
            default:
                return;
        }
    }
}
