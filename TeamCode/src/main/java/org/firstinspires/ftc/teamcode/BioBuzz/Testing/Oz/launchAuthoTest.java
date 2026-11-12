package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos.AutoTemplates.TwoPathMain;
@Disabled
@TeleOp(name = "Oz Auto Launch Test", group = "Drive")

public class launchAuthoTest extends TwoPathMain {
    @Override
    public void loop(){
        launch(1050, 1350, 3, 1);
    }
}
