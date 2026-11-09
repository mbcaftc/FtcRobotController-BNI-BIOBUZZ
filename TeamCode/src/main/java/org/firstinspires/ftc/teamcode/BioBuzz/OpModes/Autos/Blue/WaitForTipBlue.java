package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.WaitForTipMain;

@Autonomous(name = "WaitForTipBlue", group = "Drive")
public class WaitForTipBlue extends WaitForTipMain {
    //Blue Front hive tags 38-41, Blue Back hive tags 42-45 (same as BlueTeleOP)
    public WaitForTipBlue() {
        super("Blue", 38, 41, 42, 45);
    }
}
