package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Same as Far_Check_Hive, but starts turned 45 degrees toward the corner pollen (facing 315 instead of 270)
//so the turn to pick up is only 45 degrees. The turret turns to aim at the hive during the opening shot,
//then goes back to straight ahead for the rest of the auto.
//The turret must start straight ahead (0) at init, same as Start_At_Hive_Right.
@Autonomous(name = "Far_Check_Hive_45", group = "Debug")
public class FarCheckHive45 extends FarCheckHive {
    //Turned 45 degrees the 18in robot's corners reach ~12.7in from its center, so the center sits 13in off the far wall
    //(y 131) and a little right of StartFarLeft so the corners stay off the centerline (x 72). TUNE ON THE FIELD
    protected static final UniPose StartFarLeft45 = new UniPose(85, 131);

    {
        openingStart = StartFarLeft45;
        openingHeading = Math.toRadians(315);//45 degrees clockwise from 270, toward the corner pollen
        aimTurret = true;
        //The hive is straight down the field (270) from the start, which is 45 degrees clockwise of the robot's 315. TUNE ON THE FIELD
        openingTurretTarget = -45;
    }
}
