package frc.robot.processors;

import com.google.auto.service.AutoService;
import com.squareup.javapoet.*;
import frc.robot.annotations.AutoCommandFactory;
import java.io.IOException;
import java.util.Set;
import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.MirroredTypeException;

@AutoService(Processor.class)
@SupportedAnnotationTypes("frc.robot.annotations.AutoCommandFactory")
@SupportedSourceVersion(SourceVersion.RELEASE_17)
public class CommandFactoryProcessor extends AbstractProcessor {

  private static final ClassName COMMAND_CLASS =
      ClassName.get("edu.wpi.first.wpilibj2.command", "Command");
  private static final ClassName COMMANDS_CLASS =
      ClassName.get("edu.wpi.first.wpilibj2.command", "Commands");
  private static final ClassName SUPERSTRUCTURE_CLASS =
      ClassName.get("frc.robot.subsystems.superstructure", "SuperStructure");

  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
    for (Element element : roundEnv.getElementsAnnotatedWith(AutoCommandFactory.class)) {
      if (element.getKind() != ElementKind.CLASS) continue;

      TypeElement subsystemElement = (TypeElement) element;
      AutoCommandFactory annotation = subsystemElement.getAnnotation(AutoCommandFactory.class);

      TypeElement enumElement = getEnumElement(annotation);
      if (enumElement == null) {
        // Fails build if enum is unresolved
        throw new IllegalStateException(
            "Failed to resolve requestEnum for " + subsystemElement.getSimpleName());
      }

      generateFactoryClass(subsystemElement, enumElement);
    }
    return true;
  }

  private TypeElement getEnumElement(AutoCommandFactory annotation) {
    try {
      annotation.requestEnum();
    } catch (MirroredTypeException mte) {
      DeclaredType classTypeMirror = (DeclaredType) mte.getTypeMirror();
      return (TypeElement) classTypeMirror.asElement();
    }
    return null;
  }

  private void generateFactoryClass(TypeElement subsystemElement, TypeElement enumElement) {
    String packageName =
        processingEnv
            .getElementUtils()
            .getPackageOf(subsystemElement)
            .getQualifiedName()
            .toString();
    String subsystemName = subsystemElement.getSimpleName().toString();
    String factoryName = subsystemName + "Commands";

    ClassName subsystemClassName = ClassName.get(packageName, subsystemName);

    // Handle nested enums (e.g. ConveyorConstants.ConveyorRequest).
    // If the enum is enclosed in a class (not a package), we need the enclosing class name too.
    String enumPackage =
        processingEnv.getElementUtils().getPackageOf(enumElement).getQualifiedName().toString();
    Element enumEnclosing = enumElement.getEnclosingElement();
    ClassName enumClassName;
    if (enumEnclosing.getKind() == ElementKind.CLASS) {
      // Nested enum: e.g. ConveyorConstants.ConveyorRequest
      enumClassName =
          ClassName.get(
              enumPackage,
              enumEnclosing.getSimpleName().toString(),
              enumElement.getSimpleName().toString());
    } else {
      // Top-level enum
      enumClassName = ClassName.get(enumPackage, enumElement.getSimpleName().toString());
    }

    TypeSpec.Builder classBuilder =
        TypeSpec.classBuilder(factoryName)
            .addModifiers(Modifier.PUBLIC, Modifier.FINAL)
            .addMethod(
                MethodSpec.constructorBuilder()
                    .addModifiers(Modifier.PRIVATE)
                    .build()); // Prevent instantiation

    // Default Command Method
    MethodSpec defaultCommand =
        MethodSpec.methodBuilder("defaultCommand")
            .addModifiers(Modifier.PUBLIC, Modifier.STATIC)
            .returns(COMMAND_CLASS)
            .addParameter(SUPERSTRUCTURE_CLASS, "superStructure")
            .addParameter(subsystemClassName, subsystemName.toLowerCase())
            .addStatement(
                "return $T.run(() -> $L.setRequest(superStructure.get$LRequest()), $L)"
                    + "\n    .withName($S)",
                COMMANDS_CLASS,
                subsystemName.toLowerCase(),
                subsystemName,
                subsystemName.toLowerCase(),
                subsystemName + " Default Relay")
            .build();

    classBuilder.addMethod(defaultCommand);

    // Iterate over Enum constants to generate override methods
    for (Element enclosed : enumElement.getEnclosedElements()) {
      if (enclosed.getKind() == ElementKind.ENUM_CONSTANT) {
        String enumValue = enclosed.getSimpleName().toString();
        String methodName = enumValue.toLowerCase();

        MethodSpec overrideCommand =
            MethodSpec.methodBuilder(methodName)
                .addModifiers(Modifier.PUBLIC, Modifier.STATIC)
                .returns(COMMAND_CLASS)
                .addParameter(subsystemClassName, subsystemName.toLowerCase())
                .addStatement(
                    "return $T.run(() -> $L.setRequest($T.$L), $L)" + "\n    .withName($S)",
                    COMMANDS_CLASS,
                    subsystemName.toLowerCase(),
                    enumClassName,
                    enumValue,
                    subsystemName.toLowerCase(),
                    subsystemName + " Override " + enumValue)
                .build();

        classBuilder.addMethod(overrideCommand);
      }
    }

    JavaFile javaFile = JavaFile.builder(packageName, classBuilder.build()).indent("  ").build();

    try {
      javaFile.writeTo(processingEnv.getFiler());
    } catch (IOException e) {
      throw new RuntimeException("Failed to write generated factory: " + factoryName, e);
    }
  }
}
