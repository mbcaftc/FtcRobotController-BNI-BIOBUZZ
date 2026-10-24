package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.utils.Timer;
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
import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;
import org.firstinspires.ftc.teamcode.BioBuzz.pedro.ProgramConstants;

public class AutoMain extends OpMode {
    public Timer pathTimer;
    public Timer opmodeTimer;
    public Follower follower;
    @Override
    public void init() {
        setup();
    }
    public void setup(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = MainConstants.createFollower(hardwareMap);
    }

    public void loop() {}

    public static Path create(Pose start, Pose end) {
        return line(start, end).linear(start, end);
    }


}
