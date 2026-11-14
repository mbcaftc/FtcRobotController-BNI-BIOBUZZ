package org.firstinspires.ftc.teamcode.BioBuzz.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addAnalogueInput;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addCRServo;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addIMU;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addLimelight;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotor;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addMotorEx;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addOdometry;
import static org.firstinspires.ftc.teamcode.BioBuzz.Constructors.QuickRigging.addServo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.BioBuzz.Constructors.Prism.Color;
import org.firstinspires.ftc.teamcode.BioBuzz.Constructors.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.BioBuzz.Constructors.Prism.PrismAnimations;

import java.util.Collections;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TesterTeleOP", group = "Debug")
public class SimpleTeleOP extends OpMode{
    public SimpleTeleOP() {}
    public DcMotor launcherMotor;


    @Override
    public void init() {

        launcherMotor = addMotor(hardwareMap, "launcher", true, false, true);//Float so the flywheel coasts

    }
    @Override
    public void loop(){
        launcherMotor.setPower(1);
    }
}