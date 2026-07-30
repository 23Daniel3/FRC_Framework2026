package frc.robot.processors;

import static com.google.testing.compile.CompilationSubject.assertThat;

import com.google.testing.compile.Compilation;
import com.google.testing.compile.Compiler;
import com.google.testing.compile.JavaFileObjects;
import javax.tools.JavaFileObject;
import org.junit.Test;

public class CommandFactoryProcessorTest {

  @Test
  public void generatesFactoryCorrectly() {
    // Mocks for framework classes to satisfy AST resolution
    JavaFileObject commandMock =
        JavaFileObjects.forSourceLines(
            "edu.wpi.first.wpilibj2.command.Command",
            "package edu.wpi.first.wpilibj2.command;",
            "public interface Command {",
            "    Command withName(String name);",
            "}");

    JavaFileObject commandsMock =
        JavaFileObjects.forSourceLines(
            "edu.wpi.first.wpilibj2.command.Commands",
            "package edu.wpi.first.wpilibj2.command;",
            "public class Commands {",
            "    public static Command run(Runnable r, Object... req) { return null; }",
            "}");

    JavaFileObject superStructureMock =
        JavaFileObjects.forSourceLines(
            "frc.robot.subsystems.superstructure.SuperStructure",
            "package frc.robot.subsystems.superstructure;",
            "import frc.robot.subsystems.MockRequest;",
            "public class SuperStructure {",
            "    public MockRequest getMockRequest() { return null; }",
            "}");

    JavaFileObject enumFile =
        JavaFileObjects.forSourceLines(
            "frc.robot.subsystems.MockRequest",
            "package frc.robot.subsystems;",
            "public enum MockRequest { IN, OUT }");

    JavaFileObject classFile =
        JavaFileObjects.forSourceLines(
            "frc.robot.subsystems.Mock",
            "package frc.robot.subsystems;",
            "import frc.robot.annotations.AutoCommandFactory;",
            "@AutoCommandFactory(requestEnum = MockRequest.class)",
            "public class Mock {",
            "    public void setRequest(MockRequest req) {}",
            "}");

    // DO NOT inject the annotation mock here. The compiler reads it from the test classpath.
    Compilation compilation =
        Compiler.javac()
            .withProcessors(new CommandFactoryProcessor())
            .compile(commandMock, commandsMock, superStructureMock, enumFile, classFile);

    assertThat(compilation).succeeded();

    assertThat(compilation)
        .generatedSourceFile("frc.robot.subsystems.MockCommands")
        .hasSourceEquivalentTo(
            JavaFileObjects.forSourceLines(
                "frc.robot.subsystems.MockCommands",
                "package frc.robot.subsystems;",
                "",
                "import edu.wpi.first.wpilibj2.command.Command;",
                "import edu.wpi.first.wpilibj2.command.Commands;",
                "import frc.robot.subsystems.superstructure.SuperStructure;",
                "",
                "public final class MockCommands {",
                "  private MockCommands() {",
                "  }",
                "",
                "  public static Command defaultCommand(SuperStructure superStructure, Mock mock) {",
                "    return Commands.run(() -> mock.setRequest(superStructure.getMockRequest()), mock)",
                "            .withName(\"Mock Default Relay\");",
                "  }",
                "",
                "  public static Command in(Mock mock) {",
                "    return Commands.run(() -> mock.setRequest(MockRequest.IN), mock)",
                "            .withName(\"Mock Override IN\");",
                "  }",
                "",
                "  public static Command out(Mock mock) {",
                "    return Commands.run(() -> mock.setRequest(MockRequest.OUT), mock)",
                "            .withName(\"Mock Override OUT\");",
                "  }",
                "}"));
  }
}
