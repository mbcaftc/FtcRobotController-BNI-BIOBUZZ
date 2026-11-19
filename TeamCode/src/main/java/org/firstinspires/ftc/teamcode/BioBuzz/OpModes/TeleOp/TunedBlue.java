package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.TeleOp;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueTunedTeleOP", group = "Test")
public class TunedBlue extends TunedTeleOP{
    @Override
    public void init(){
        blueSide = true;
        super.init();
    }
}
