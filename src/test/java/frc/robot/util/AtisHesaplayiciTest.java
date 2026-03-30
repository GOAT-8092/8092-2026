package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

class AtisHesaplayiciTest {
    @Test
    @Tag("fast")
    void belowTableUsesFirstValue() {
        assertEquals(3800.0, AtisHesaplayici.hesaplaHedefRpm(0.8), 1e-9);
    }

    @Test
    @Tag("fast")
    void aboveTableUsesLastValue() {
        assertEquals(5400.0, AtisHesaplayici.hesaplaHedefRpm(6.5), 1e-9);
    }

    @Test
    @Tag("fast")
    void interpolatesBetweenPoints() {
        assertEquals(4850.0, AtisHesaplayici.hesaplaHedefRpm(2.4), 1e-9);
    }
}
