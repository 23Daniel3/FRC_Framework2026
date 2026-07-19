#!/usr/bin/env python3
"""Gerador de subsistemas do template.

Uso:
    python3 tools/new_subsystem.py Climber
    python3 tools/new_subsystem.py ArmPivot --vendor talonfx

Gera robot/subsystems/<nome>/ com as 4 classes do padrao Request -> State
(Constants, IO, IOHardware, Subsystem), prontas para compilar. Depois de gerar:

  1. Ajuste MOTOR_ID e MOTOR_CONFIG nas Constants;
  2. Modele os enums Request e State do mecanismo;
  3. Configure a FSM e os bindRequest no construtor do subsistema;
  4. Instancie no RobotContainer com o IO certo por Constants.currentMode;
  5. Faca a SuperStructure escrever os requests novos nos onEnter da FSM geral.
"""

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
EXAMPLE = ROOT / "robot" / "subsystems" / "example"

VENDOR_SNIPPETS = {
    "sparkmax": (
        "import com.revrobotics.spark.SparkLowLevel.MotorType;\n"
        "import frc.lib.interfaces.motor.MotorIOSparkMax;",
        '        new MotorIOSparkMax(\n'
        '            "{Name}Motor",\n'
        "            {Name}Constants.MOTOR_ID,\n"
        "            MotorType.kBrushless,\n"
        "            {Name}Constants.MOTOR_CONFIG);",
    ),
    "sparkflex": (
        "import com.revrobotics.spark.SparkLowLevel.MotorType;\n"
        "import frc.lib.interfaces.motor.MotorIOSparkFlex;",
        '        new MotorIOSparkFlex(\n'
        '            "{Name}Motor",\n'
        "            {Name}Constants.MOTOR_ID,\n"
        "            MotorType.kBrushless,\n"
        "            {Name}Constants.MOTOR_CONFIG);",
    ),
    "talonfx": (
        "import com.ctre.phoenix6.CANBus;\n"
        "import frc.lib.interfaces.motor.MotorIOTalonFX;",
        '        new MotorIOTalonFX(\n'
        '            "{Name}Motor",\n'
        "            {Name}Constants.MOTOR_ID,\n"
        "            new CANBus(),\n"
        "            {Name}Constants.MOTOR_CONFIG);",
    ),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="Nome do subsistema em PascalCase (ex.: Climber)")
    parser.add_argument(
        "--vendor",
        choices=sorted(VENDOR_SNIPPETS),
        default="sparkmax",
        help="Controlador de motor do IOHardware gerado (default: sparkmax)",
    )
    args = parser.parse_args()

    name = args.name
    if not name[0].isupper() or not name.isidentifier():
        print(f"erro: '{name}' deve ser um identificador PascalCase (ex.: Climber)")
        return 1

    lower = name.lower()
    dest = ROOT / "robot" / "subsystems" / lower
    if dest.exists():
        print(f"erro: {dest} ja existe — remova ou escolha outro nome.")
        return 1
    if not EXAMPLE.exists():
        print(f"erro: pasta de referencia nao encontrada: {EXAMPLE}")
        return 1

    dest.mkdir(parents=True)
    imports, ctor = VENDOR_SNIPPETS[args.vendor]

    for src in sorted(EXAMPLE.glob("Example*.java")):
        text = src.read_text(encoding="utf-8")

        if src.name == "ExampleIOHardware.java" and args.vendor != "sparkmax":
            text = text.replace(
                "import com.revrobotics.spark.SparkLowLevel.MotorType;\n"
                "import frc.lib.interfaces.motor.MotorController;\n"
                "import frc.lib.interfaces.motor.MotorIO;\n"
                "import frc.lib.interfaces.motor.MotorIOSparkMax;",
                "import frc.lib.interfaces.motor.MotorController;\n"
                "import frc.lib.interfaces.motor.MotorIO;\n" + imports,
            )
            text = text.replace(
                '        new MotorIOSparkMax(\n'
                '            "ExampleMotor",\n'
                "            ExampleConstants.MOTOR_ID,\n"
                "            MotorType.kBrushless,\n"
                "            ExampleConstants.MOTOR_CONFIG);",
                ctor.replace("{Name}", "Example"),
            )

        text = text.replace("Example", name).replace("subsystems.example", f"subsystems.{lower}")
        out = dest / src.name.replace("Example", name)
        out.write_text(text, encoding="utf-8")
        print(f"criado: {out.relative_to(ROOT)}")

    print(f"\n{name} gerado. Proximos passos: ver docstring deste script.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
