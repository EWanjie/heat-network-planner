package ru.heatplanner.plan;

import org.junit.jupiter.api.Test;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Чтение большого файла в два прохода: ограничения вне рабочей области не загружаются. */
class PlanInputAreaTest {

    private static String point(String id, String type, double lon, double lat, String extra) {
        return "{\"type\":\"Feature\",\"properties\":{\"id\":" + id + ",\"object_type\":\"" + type + "\"" + extra
                + "},\"geometry\":{\"type\":\"Point\",\"coordinates\":[" + lon + "," + lat + "]}}";
    }

    private static String building(String id, double lon, double lat) {
        double d = 0.0002;
        return "{\"type\":\"Feature\",\"properties\":{\"id\":" + id + ",\"object_type\":\"restriction\",\"restriction_type\":\"oks\"},"
                + "\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[" + lon + "," + lat + "],[" + (lon + d) + "," + lat + "],["
                + (lon + d) + "," + (lat + d) + "],[" + lon + "," + (lat + d) + "],[" + lon + "," + lat + "]]]}}";
    }

    @Test
    void restrictionsFarFromTheNetworkAreNotLoaded() throws Exception {
        String file = "{\"type\":\"FeatureCollection\",\"features\":["
                + point("1", "source", 37.6, 55.75, "") + ","
                + "{\"type\":\"Feature\",\"properties\":{\"id\":2,\"object_type\":\"heat_network\",\"diameter\":100},"
                + "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[[37.6,55.75],[37.601,55.75]]}},"
                + point("3", "oks_connection_point", 37.6005, 55.7515, ",\"flow_tph\":10") + ","
                + building("4", 37.6003, 55.7508) + ","      // рядом
                + building("5", 37.9, 55.9) + "]}";      // в десятках километров
        Path tmp = Files.createTempFile("area-", ".geojson");
        try {
            Files.write(tmp, file.getBytes(StandardCharsets.UTF_8));
            PlanInput in = PlanInput.read(tmp, null, RuleSet.strict(), false);
            long oks = in.obstacles.stream().filter(o -> "oks".equals(o.rule.type)).count();
            assertEquals(1, oks, "остаться должно только здание рядом с сетью");
            assertEquals(1, in.targets.size());
            assertEquals(1, in.segments.size());
            assertTrue(in.diagnostics.stream().anyMatch(d -> d.contains("вне рабочей области")));
        } finally {
            Files.deleteIfExists(tmp);
        }
    }
}