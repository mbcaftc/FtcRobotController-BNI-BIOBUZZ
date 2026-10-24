package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue_CloseLeft_One_A", group = "Drive")
public class Blue_CloseLeft_One_A extends BlueMain{
    Path StartToShoot;
    Path ShootToPark;
    public void pathGen(){
        StartToShoot = createLinearPath(StartCloseLeft, Quadrant3Center, 90, 90);
    }
    @Override
    public void init(){
        setup();
        follower.setPose(StartCloseLeft);
        follower.update();

    }

    @Override
    public void loop(){

    }
}
