package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsBlue;

// Wow, Ayden is in the dictionary but not Adyn??
@Autonomous(name = "Adyn Blue", preselectTeleOp = "TeleOp")
public class ManualAutoBlue extends ManualAuto {
    @Override
    public Paths getPaths() {
        return new PathsBlue(pedro);
    }
}
