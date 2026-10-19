package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;



import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoMain extends OpMode {
    @Override
    public void init() {

    }
    public void loop() {}

    public static Path create(Pose start, Pose end) {
        return (new AtomicPath(new Line(start, end)).linear(start, end));
    }

}
