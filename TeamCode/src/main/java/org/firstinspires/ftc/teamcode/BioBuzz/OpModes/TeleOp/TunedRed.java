package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.TeleOp;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RedTunedTeleOP", group = "Test")
public class TunedRed extends TunedTeleOP{
    @Override
    public void init(){
        blueSide = false;
        super.init();
    }
}
