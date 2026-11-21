package org.ciprix.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Generates the command factory of a state subsystem at compile time.
 *
 * <p>For a subsystem {@code Intake} annotated with {@code @AutoCommandFactory(requestEnum =
 * IntakeRequest.class)}, the {@code ciprix-processor} annotation processor writes {@code
 * IntakeCommands} in the same package with:
 *
 * <ul>
 *   <li>{@code defaultCommand(requestSource, intake)}: the relay used as the default command of the
 *       subsystem. Every cycle it copies the request computed by the SuperStructure.
 *   <li>{@code defaultCommand(superStructure, intake)}: same relay, reading {@code
 *       superStructure.getIntakeRequest()}. Generated when the SuperStructure class is known (see
 *       {@link #superStructure()}).
 *   <li>One command per request ({@code in(intake)}, {@code out(intake)}, ...): the manual override
 *       used by the copilot. It holds the request only while it is scheduled and interrupts only
 *       the relay of this subsystem, which resumes when the button is released.
 * </ul>
 *
 * <p>The annotation is discarded after compilation and has no runtime cost.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.SOURCE)
public @interface AutoCommandFactory {
  /** Enum with the requests accepted by the subsystem. */
  Class<? extends Enum<?>> requestEnum();

  /**
   * SuperStructure class that owns the request of this subsystem. It must expose {@code
   * get<Subsystem>Request()}.
   *
   * <p>When omitted, the processor looks for the class named by the {@code ciprix.superStructure}
   * compiler option, which defaults to {@code frc.robot.subsystems.superstructure.SuperStructure}.
   * If that class does not exist, only the {@code Supplier} based relay is generated.
   */
  Class<?> superStructure() default void.class;
}
