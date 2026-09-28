package org.firstinspires.ftc.teamcode.BioBuzz.Testing.Oz;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;

import java.util.ArrayList;
import java.util.List;

public class EasyCreate {
    public static List<Path> create(Pose... points) {
        List<Path> paths = new ArrayList<>();
        for (int i = 0; i < points.length - 1; i++) {
            Pose start = points[i];
            Pose end = points[i + 1];
            // Fuses atomic pathing to create a path between points atomic Path is on pedro for documentation
            paths.add(new AtomicPath(new Line(start, end)).linear(start, end));
        }
        return paths;
    }
}
