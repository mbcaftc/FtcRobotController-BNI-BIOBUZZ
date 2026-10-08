package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class PoseMain extends FunctionMain {
     protected static final PoseFactory poseFactory = PoseFactory.degrees();

     protected static final Pose start = poseFactory.of(56, 8, 90);
     protected static final Pose path1 = poseFactory.of(27.2335, 36.7775, 180);
     protected static final Pose point2 = poseFactory.of(25.625, 77.614, -87.7443);
     protected static final Pose point3 = poseFactory.of(26.9547, 113.158, -92.1424);
}
