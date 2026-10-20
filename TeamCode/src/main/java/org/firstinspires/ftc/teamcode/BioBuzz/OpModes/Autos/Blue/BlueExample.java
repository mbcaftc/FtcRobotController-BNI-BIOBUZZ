package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.pedro.ProgramConstants;

@Autonomous(name = "Pedro Test v1", group = "Drive")
public class BlueExample extends BlueMain{
    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = ProgramConstants.createFollower(hardwareMap);
        follower.setPose(start);
        follower.update();
        pathGen();
    }
}
