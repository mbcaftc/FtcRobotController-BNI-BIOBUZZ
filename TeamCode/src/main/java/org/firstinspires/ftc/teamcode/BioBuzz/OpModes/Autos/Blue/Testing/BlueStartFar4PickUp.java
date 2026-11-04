package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.Testing;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.FarHiveTip;
import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.MovmentMain;
import org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew.AndrewPose;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
@Autonomous(name = "Start_Far_PickUp", group = "Drive")
public class BlueStartFar4PickUp extends MovmentMain {
    Path start_to_park;


    @Override
    public void init(){
        //AutoMain init
        setup();
        initHardWare();
        //Set BlueMain points (BEFORE buildPaths())

        //pre PickUp
        Pose start = new Pose(StartFarLeft.x(), StartFarLeft.y(), 0);


        //PickUp


        buildPaths();
        //Start the follower
        follower.setPose(new Pose(start.x(), start.y(), start.heading()));
        follower.update();
    }
    @Override
    public void pathLogic(){
        launch(960, 1225, 3, 1);
        if (opmodeTimer.seconds() > 10) {
            cutPower();
            movePath(start_to_park, pathingState.PARK);
        }
    }
}

