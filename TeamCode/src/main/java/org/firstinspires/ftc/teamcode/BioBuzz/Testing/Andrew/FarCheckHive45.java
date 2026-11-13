package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Same as Far_Check_Hive, but starts turned 45 degrees toward the corner pollen (facing 315 instead of 270)
//so the turn to pick up is only 45 degrees. The turret turns to aim at the hive during the opening shot,
//then goes back to straight ahead for the rest of the auto.
//The turret must start straight ahead (0) at init, same as Start_At_Hive_Right.
@Autonomous(name = "Far_Check_Hive_45", group = "Debug")
public class FarCheckHive45 extends FarCheckHive {
    //Robot is 17in front to back, 15in side to side. Facing 315, the corners sit at (+-11.31, -+0.71) and (-+0.71, +-11.31)
    //from the center: (8.5 + 7.5) * cos45 = 11.31 and (8.5 - 7.5) * cos45 = 0.71
    //  - Back-left corner touching the far wall: center y = 144 - 11.31 = 132.7
    //  - Back-right corner almost on the midline (0.5in gap): center x = 72 + 11.31 + 0.5 = 83.8
    //Assumes the pose is the center of the robot's frame. TUNE ON THE FIELD
    protected static final UniPose StartFarLeft45 = new UniPose(83.8, 132.7);

    {
        openingStart = StartFarLeft45;
        openingHeading = Math.toRadians(315);//45 degrees clockwise from 270, toward the corner pollen
        aimTurret = true;
        //The hive center (84, 72) is straight down the field (270.2) from (83.8, 132.7), which is 44.8 degrees clockwise
        //of the robot's 315. Rounded to 45. TUNE ON THE FIELD
        openingTurretTarget = -45;
    }
}
