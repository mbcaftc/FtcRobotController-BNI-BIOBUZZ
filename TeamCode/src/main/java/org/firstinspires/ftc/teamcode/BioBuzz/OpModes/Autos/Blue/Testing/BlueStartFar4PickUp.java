package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.Testing;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.paths.Path;
@Autonomous(name = "Start_Far_PickUp_oz", group = "Drive")
public class BlueStartFar4PickUp extends MovmentMain {
    Path start_to_park;
    ScoringState CurrentState;
    enum ScoringState {
        Waiting,
        Shooting,
        GoingToPickUp,
        Pickup,
        Park
    }

    Path prePickup;
    Path Pickup;
    Path Park;

    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildConstantPaths())

        //pre PickUp
        prePickup = buildConstantPaths(StartFarLeft.pose(), PreLooseBallsPickup.pose());
        //PickUp
        Pickup = buildConstantPaths(PreLooseBallsPickup.pose(), LooseBallsPickup.pose(), PreLooseBallsPickup.pose());
        //park
        Park = buildConstantPaths(PreLooseBallsPickup.pose(), Quadrant1Center.setDeg(90), ParkPoseA.setDeg(180));
        //Start the follower


        follower.setPose(StartFarLeft.pose());
        follower.update();
        CurrentState = ScoringState.Waiting;//Waiting nothing is happening started in start
    }
    @Override
    public void start(){
        CurrentState = ScoringState.Shooting;//Begins shooting
    }
    @Override
    public void pathLogic(){
        switch(CurrentState){
            case Waiting:
                break;
            case Shooting:
                launch(960, 1225, 3, 1);
                if (opmodeTimer.seconds() > 10) CurrentState = ScoringState.GoingToPickUp;
                break;
            case GoingToPickUp:
                cutPower();
                movePath(prePickup, pathingState.LOOP);
                CurrentState = ScoringState.GoingToPickUp;
                break;
            case Pickup:
                intake(1);
                movePath(Pickup, pathingState.LOOP);
                intake(0);
                CurrentState = ScoringState.Park;
                break;
            case Park:
                movePath(Park, pathingState.PARK);
        }
    }
}

