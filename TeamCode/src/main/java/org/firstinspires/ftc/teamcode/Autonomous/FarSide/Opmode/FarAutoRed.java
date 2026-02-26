package org.firstinspires.ftc.teamcode.Autonomous.FarSide.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.Paths;
import org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths.PathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPathsBlue;
import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPathsRed;

//efn
@Autonomous(name = "Keir Far Red", preselectTeleOp = "TeleOp")
public class FarAutoRed extends FarAuto {
    @Override
    public FarPaths getPaths() {
        return new FarPathsRed(pedro);
    }
}
