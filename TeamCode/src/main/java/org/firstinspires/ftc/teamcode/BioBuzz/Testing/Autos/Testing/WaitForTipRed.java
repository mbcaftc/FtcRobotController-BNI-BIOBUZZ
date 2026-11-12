package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Autos.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "WaitForTipRed", group = "Drive")
public class WaitForTipRed extends WaitForTipMain {
    //Red Front hive tags 34-37, Red Back hive tags 30-33 (same as RedTeleOP)
    public WaitForTipRed() {
        super("Red", 34, 37, 30, 33);
    }
}
