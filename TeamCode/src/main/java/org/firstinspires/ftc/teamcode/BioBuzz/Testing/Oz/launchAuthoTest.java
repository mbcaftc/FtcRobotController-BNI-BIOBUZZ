package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.Blue.BlueMain;

@TeleOp(name = "Oz Auto Launch Test", group = "Drive")

public class launchAuthoTest extends BlueMain {
    @Override
    public void loop(){
        launch(1000, 1);
    }
}
