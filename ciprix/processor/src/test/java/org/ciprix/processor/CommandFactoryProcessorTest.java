package org.ciprix.processor;

import static com.google.common.truth.Truth.assertThat;
import static com.google.testing.compile.CompilationSubject.assertThat;
import static org.junit.Assert.assertEquals;

import com.google.testing.compile.Compilation;
import com.google.testing.compile.Compiler;
import com.google.testing.compile.JavaFileObjects;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import javax.tools.JavaFileObject;
import org.junit.Test;

public class CommandFactoryProcessorTest {

  // Mocks for framework classes to satisfy AST resolution
  private static final JavaFileObject COMMAND =
      JavaFileObjects.forSourceLines(
          "edu.wpi.first.wpilibj2.command.Command",
          "package edu.wpi.first.wpilibj2.command;",
          "public interface Command {",
          "    Command withName(String name);",
          "}");

  private static final JavaFileObject COMMANDS =
      JavaFileObjects.forSourceLines(
          "edu.wpi.first.wpilibj2.command.Commands",
          "package edu.wpi.first.wpilibj2.command;",
          "public class Commands {",
          "    public static Command run(Runnable r, Object... req) { return null; }",
          "}");

  private static final JavaFileObject SUPERSTRUCTURE =
      JavaFileObjects.forSourceLines(
          "frc.robot.subsystems.superstructure.SuperStructure",
          "package frc.robot.subsystems.superstructure;",
          "import frc.robot.subsystems.MockRequest;",
          "public class SuperStructure {",
          "    public MockRequest getMockRequest() { return null; }",
          "}");

  private static final JavaFileObject REQUEST_ENUM =
      JavaFileObjects.forSourceLines(
          "frc.robot.subsystems.MockRequest",
          "package frc.robot.subsystems;",
          "public enum MockRequest { IN, OUT }");

  private static final JavaFileObject SUBSYSTEM =
      JavaFileObjects.forSourceLines(
          "frc.robot.subsystems.Mock",
          "package frc.robot.subsystems;",
          "import org.ciprix.annotations.AutoCommandFactory;",
          "@AutoCommandFactory(requestEnum = MockRequest.class)",
          "public class Mock {",
          "    public void setRequest(MockRequest req) {}",
          "}");

  /**
   * The real annotation, read from the core module so this test can never drift from it. The
   * processor itself only knows the annotation by name.
   */
  private static JavaFileObject annotation() {
    Path source =
        Path.of(
            System.getProperty("ciprix.annotationSource", "../core/src/main/java"),
            "org/ciprix/annotations/AutoCommandFactory.java");
    try {
      return JavaFileObjects.forSourceString(
          "org.ciprix.annotations.AutoCommandFactory",
          Files.readString(source, StandardCharsets.UTF_8));
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private static Compilation compile(JavaFileObject... sources) {
    List<JavaFileObject> all = new ArrayList<>(List.of(annotation(), COMMAND, COMMANDS));
    all.addAll(List.of(sources));
    return Compiler.javac().withProcessors(new CommandFactoryProcessor()).compile(all);
  }

  private static String generated(Compilation compilation, String qualifiedName) {
    try {
      return compilation
          .generatedSourceFile(qualifiedName)
          .orElseThrow()
          .getCharContent(true)
          .toString();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  @Test
  public void generatesFactoryCorrectly() {
    Compilation compilation = compile(SUPERSTRUCTURE, REQUEST_ENUM, SUBSYSTEM);

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
                "import java.util.function.Supplier;",
                "",
                "public final class MockCommands {",
                "  private MockCommands() {",
                "  }",
                "",
                "  public static Command defaultCommand(Supplier<MockRequest> requestSource,",
                "      Mock mock) {",
                "    return Commands.run(() -> mock.setRequest(requestSource.get()), mock)",
                "        .withName(\"Mock Default Relay\");",
                "  }",
                "",
                "  public static Command defaultCommand(SuperStructure superStructure, Mock mock)"
                    + " {",
                "    return defaultCommand(superStructure::getMockRequest, mock);",
                "  }",
                "",
                "  public static Command override(Mock mock, MockRequest request) {",
                "    return Commands.run(() -> mock.setRequest(request), mock)",
                "        .withName(\"Mock Override \" + request);",
                "  }",
                "",
                "  public static Command in(Mock mock) {",
                "    return Commands.run(() -> mock.setRequest(MockRequest.IN), mock)",
                "        .withName(\"Mock Override IN\");",
                "  }",
                "",
                "  public static Command out(Mock mock) {",
                "    return Commands.run(() -> mock.setRequest(MockRequest.OUT), mock)",
                "        .withName(\"Mock Override OUT\");",
                "  }",
                "}"));
  }

  @Test
  public void withoutSuperStructureOnlyTheSupplierRelayIsGenerated() {
    Compilation compilation = compile(REQUEST_ENUM, SUBSYSTEM);

    assertThat(compilation).succeeded();
    String source = generated(compilation, "frc.robot.subsystems.MockCommands");
    assertThat(source).contains("defaultCommand(Supplier<MockRequest> requestSource");
    assertThat(source).doesNotContain("SuperStructure");
  }

  @Test
  public void explicitSuperStructureClassIsUsed() {
    JavaFileObject brain =
        JavaFileObjects.forSourceLines(
            "team.Brain",
            "package team;",
            "import frc.robot.subsystems.MockRequest;",
            "public class Brain {",
            "    public MockRequest getMockRequest() { return null; }",
            "}");
    JavaFileObject subsystem =
        JavaFileObjects.forSourceLines(
            "frc.robot.subsystems.Mock",
            "package frc.robot.subsystems;",
            "import org.ciprix.annotations.AutoCommandFactory;",
            "@AutoCommandFactory(requestEnum = MockRequest.class, superStructure ="
                + " team.Brain.class)",
            "public class Mock {",
            "    public void setRequest(MockRequest req) {}",
            "}");

    Compilation compilation = compile(brain, REQUEST_ENUM, subsystem);

    assertThat(compilation).succeeded();
    assertThat(generated(compilation, "frc.robot.subsystems.MockCommands"))
        .contains("defaultCommand(Brain superStructure, Mock mock)");
  }

  @Test
  public void nestedEnumAndKeywordConstantsCompile() {
    JavaFileObject constants =
        JavaFileObjects.forSourceLines(
            "frc.robot.subsystems.MockConstants",
            "package frc.robot.subsystems;",
            "public class MockConstants {",
            "    public enum MockRequest { RUN_SLOW, DEFAULT, NEW, OVERRIDE }",
            "}");
    JavaFileObject subsystem =
        JavaFileObjects.forSourceLines(
            "frc.robot.subsystems.Mock",
            "package frc.robot.subsystems;",
            "import org.ciprix.annotations.AutoCommandFactory;",
            "@AutoCommandFactory(requestEnum = MockConstants.MockRequest.class)",
            "public class Mock {",
            "    public void setRequest(MockConstants.MockRequest req) {}",
            "}");

    Compilation compilation = compile(constants, subsystem);

    assertThat(compilation).succeeded();
    String source = generated(compilation, "frc.robot.subsystems.MockCommands");
    assertThat(source).contains("Command run_slow(Mock mock)");
    assertThat(source).contains("Command defaultRequest(Mock mock)");
    assertThat(source).contains("Command newRequest(Mock mock)");
    assertThat(source).contains("Command overrideRequest(Mock mock)");
  }

  @Test
  public void methodNamesKeepTheLowerCaseConvention() {
    assertEquals("in", CommandFactoryProcessor.methodNameFor("IN"));
    assertEquals("run_slow", CommandFactoryProcessor.methodNameFor("RUN_SLOW"));
    assertEquals("defaultRequest", CommandFactoryProcessor.methodNameFor("DEFAULT"));
    assertEquals("switchRequest", CommandFactoryProcessor.methodNameFor("SWITCH"));
  }
}
