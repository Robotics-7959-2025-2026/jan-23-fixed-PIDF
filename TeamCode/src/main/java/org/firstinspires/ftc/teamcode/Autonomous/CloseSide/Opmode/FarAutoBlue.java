package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Opmode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsFarRed;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsFarBlue;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsRed;

@Autonomous(name = "Red Far", preselectTeleOp = "TeleOp")

public class FarAutoBlue extends FarAuto {
    @Override
    public Paths getPaths() {
        return new PathsFarBlue(pedro);
    }
}
