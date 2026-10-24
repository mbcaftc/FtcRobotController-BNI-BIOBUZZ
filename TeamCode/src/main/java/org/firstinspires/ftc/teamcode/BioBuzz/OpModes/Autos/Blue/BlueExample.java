package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.pedro.ProgramConstants;

@Autonomous(name = "Pedro Test v1", group = "Drive")
public class BlueExample extends BlueMain{
    @Override
    public void init(){
        setup();
        follower.setPose(pose1);
        follower.update();
        pathGen();
    }
}
