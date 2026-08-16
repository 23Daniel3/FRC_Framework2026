package frc.lib.interfaces.encoder;

/**
 * No-Op implementation (Null Object Pattern) of {@link EncoderIO}. Used to avoid
 * NullPointerExceptions when a mechanism doesn't have (or doesn't yet have) a dedicated encoder —
 * e.g. before you've decided whether an arm needs an external sensor at all.
 */
public class EncoderIONone implements EncoderIO {}