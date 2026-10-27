package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.Autos;

import static com.pedropathing.api.Paths.line;


import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addCRServo;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotor;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotorEx;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addServo;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BioBuzz.pedro.MainConstants;

public class AutoMain extends OpMode {
    public Timer pathTimer;
    public Timer opmodeTimer;
    public Follower follower;

    public DcMotor intakeMotor;
    public DcMotor intakeMotorTwo;
    public DcMotorEx launcherMotor;
    public CRServo transferServo;
    public CRServo spinNoBackServo;
    public Servo flowerAxon;
    public CRServo turretServo;
    public CRServo pollenServo;
    public void initHardWare(){
        intakeMotor = addMotor(hardwareMap, "intake", true, true);
        intakeMotorTwo= addMotor(hardwareMap, "intakeTwo", true, true);
        launcherMotor = addMotorEx(hardwareMap, "launcher", true, false, true);//Float so the flywheel coasts

        //Init servos
        transferServo = addCRServo(hardwareMap, "transfer", true);
        turretServo = addCRServo(hardwareMap, "turret", true);
        pollenServo = addCRServo(hardwareMap, "pollen", true);
        spinNoBackServo = addCRServo(hardwareMap, "spinNoBack", true);
        flowerAxon = addServo(hardwareMap, "flowerAxon", true);
    }
    public void setup(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = MainConstants.createFollower(hardwareMap);
    }

public void init(){}//do not add anything needed for opmode to compile

    public void loop() {}

    public static Path createLinearPath(Pose start, Pose end, double headingStart, double headingEnd) {
        Pose a = new Pose(start.x(), start.y(), headingStart);
        Pose b = new Pose(end.x(), end.y(), headingEnd);
        return line(a, b).linear(a, b);
    }
    public void launch(double laucnhSpeed, double servoPower){
        launcherMotor.setVelocity(laucnhSpeed);
        transferServo.setPower(servoPower);
        pollenServo.setPower(-servoPower);
        spinNoBackServo.setPower(-servoPower);
    }




}
