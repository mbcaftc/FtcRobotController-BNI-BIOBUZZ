package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.Testing;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.FarHiveTip;
import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;
import org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.AndrewPose;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import android.widget.Switch;

import com.pedropathing.math.Pose;
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
        //Set BlueMain points (BEFORE buildPaths())

        //pre PickUp
        Pose start = new Pose(StartFarLeft.x(), StartFarLeft.y(), 0);
        prePickup = buildPaths(StartFarLeft, PreLooseBallsPickup);
        //PickUp
        Pickup = buildPaths(PreLooseBallsPickup.withHeading(0), LooseBallsPickup.withHeading(0), PreLooseBallsPickup.withHeading(0));
        //park
        Park = buildPaths(PreLooseBallsPickup.withHeading(0), Quadrant1Center.withHeading(90), ParkPoseA.withHeading(180));
        //Start the follower


        follower.setPose(new Pose(start.x(), start.y(), start.heading()));
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

