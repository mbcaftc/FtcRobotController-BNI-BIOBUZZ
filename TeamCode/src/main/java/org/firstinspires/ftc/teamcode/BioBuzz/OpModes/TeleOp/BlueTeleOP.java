package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueTeleOP", group = "Drive")
public class BlueTeleOP extends AllianceTeleOP {
    //Blue Front hive tags 38-41, Blue Back hive tags 42-45
    public BlueTeleOP() {
        super(true, 38, 41, 42, 45);
    }
}
