package frc.robot.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target(ElementType.TYPE)
@Retention(RetentionPolicy.SOURCE) // Discarded after compilation, zero runtime overhead
public @interface AutoCommandFactory {
  Class<? extends Enum<?>> requestEnum();
}
