package ru.heatplanner;

/**
 * Соответствие типа геометрии типу объекта (таблица 2.1 технического приложения).
 *
 * Форму координат, диапазон долготы и широты и замкнутость колец проверяет {@link GeoJsonGeometryValidator};
 * здесь только то, чего он не знает: какой геометрии требует каждый object_type.
 */
final class GeometryChecker {

    private GeometryChecker() {
    }

    /**
     * Описание проблемы (без точки в конце) или null, если тип геометрии подходит типу объекта.
     * Ограничения (restriction) могут иметь любую геометрию, поэтому для них проверки нет.
     */
    static String checkKind(String objectType, String geometryType) {
        switch (objectType) {
            case "source":
            case "heat_chamber":
            case "oks_connection_point":
                return geometryType.equals("Point") ? null : mismatch(objectType, "Point", geometryType);
            case "heat_network":
                return geometryType.equals("LineString") ? null : mismatch(objectType, "LineString", geometryType);
            case "oks_future":
            case "oks_existing":
                return geometryType.equals("Polygon") || geometryType.equals("MultiPolygon")
                        ? null : mismatch(objectType, "Polygon или MultiPolygon", geometryType);
            default:
                return null;
        }
    }

    private static String mismatch(String objectType, String expected, String actual) {
        return "объект типа " + objectType + " должен иметь геометрию " + expected + ", а не "
                + (actual.isEmpty() ? "пустую" : actual);
    }
}