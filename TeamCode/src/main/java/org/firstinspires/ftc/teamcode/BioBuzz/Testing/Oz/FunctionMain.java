package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class FunctionMain extends OpMode {
    @Override
    public void init() {} //Acts on all autos run
    @Override
    public void loop() {}//Acts on all autos run

    public static Path create(Pose start, Pose end) {
        return (new AtomicPath(new Line(start, end)).linear(start, end));
    }
}
