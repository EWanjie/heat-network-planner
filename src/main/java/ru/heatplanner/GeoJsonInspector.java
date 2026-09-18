package ru.heatplanner;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/** Первый этап: чтение и инвентаризация, а не полный валидатор задания. */
public class GeoJsonInspector {

    /** Ошибка структуры или содержимого GeoJSON — не ошибка ввода-вывода. */
    public static class GeoJsonValidationException extends IOException {
        public GeoJsonValidationException(String message) {
            super(message);
        }
    }

    /** Результат инвентаризации — то, что уйдёт в JSON-ответ REST-эндпоинта. */
    public static class InspectionResult {
        public long fileSizeBytes;
        public long totalObjects;
        public Map<String, Long> typeCounts;
        public Map<String, Long> restrictionCounts;
        public ExistingNetworkBuilder.NetworkSummary network;
    }

    public static void main(String[] args) {
        if (args.length != 1) {
            System.err.println("Укажите путь к GeoJSON в Program arguments, в двойных кавычках.");
            System.exit(1);
        }
        try {
            InspectionResult result = inspect(Path.of(args[0]));
            printResult(args[0], result);
        } catch (IOException | IllegalArgumentException e) {
            System.err.println("Файл не прочитан полностью. Ошибка: " + e.getMessage());
            System.exit(1);
        }
    }

    public static InspectionResult inspect(Path path) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.enable(JsonParser.Feature.STRICT_DUPLICATE_DETECTION);
        Map<String, Long> types = new TreeMap<>();
        Map<String, Long> restrictions = new TreeMap<>();
        Set<String> seenIds = new HashSet<>();
        ExistingNetworkBuilder networkBuilder = new ExistingNetworkBuilder();
        long total = 0;
        boolean featuresFound = false;
        String rootType = null;

        try (JsonParser parser = mapper.getFactory().createParser(path.toFile())) {
            require(parser.nextToken() == JsonToken.START_OBJECT, "Корень JSON должен быть объектом.");
            while (parser.nextToken() != JsonToken.END_OBJECT) {
                require(parser.currentToken() == JsonToken.FIELD_NAME, "Ожидалось поле корневого объекта.");
                String field = parser.currentName();
                require(parser.nextToken() != null, "Неожиданный конец файла.");
                if ("type".equals(field)) {
                    require(parser.currentToken() == JsonToken.VALUE_STRING, "Корневой type должен быть строкой.");
                    rootType = parser.getText();
                } else if ("features".equals(field)) {
                    featuresFound = true;
                    require(parser.currentToken() == JsonToken.START_ARRAY, "features должен быть массивом.");
                    while (parser.nextToken() != JsonToken.END_ARRAY) {
                        require(parser.currentToken() == JsonToken.START_OBJECT,
                                "Feature #" + (total + 1) + " должен быть объектом.");
                        JsonNode feature = mapper.readTree(parser);
                        require("Feature".equals(feature.path("type").asText()),
                                "Неверный type у Feature #" + (total + 1));

                        JsonNode geometry = feature.path("geometry");
                        require(geometry.isObject(), "Нет geometry у Feature #" + (total + 1));

                        JsonNode properties = feature.path("properties");
                        require(properties.isObject(), "Нет properties у Feature #" + (total + 1));

                        // id может прийти и строкой, и числом (так отдают многие ГИС-инструменты) —
                        // принимаем оба варианта и приводим к строке.
                        JsonNode idNode = properties.path("id");
                        require(!idNode.isMissingNode() && !idNode.isNull()
                                        && (idNode.isTextual() || idNode.isNumber())
                                        && !idNode.asText().isBlank(),
                                "Нет id у Feature #" + (total + 1));
                        String id = idNode.asText();
                        require(seenIds.add(id), "Повторяющийся id \"" + id + "\" у Feature #" + (total + 1));

                        JsonNode type = properties.path("object_type");
                        require(type.isTextual() && !type.asText().isBlank(),
                                "Нет строкового object_type у Feature #" + (total + 1));
                        types.merge(type.asText(), 1L, Long::sum);
                        if ("restriction".equals(type.asText())) {
                            JsonNode restriction = properties.path("restriction_type");
                            require(restriction.isTextual() && !restriction.asText().isBlank(),
                                    "Нет restriction_type у Feature #" + (total + 1));
                            restrictions.merge(restriction.asText(), 1L, Long::sum);
                        }
                        networkBuilder.add(feature, type.asText(), id, total + 1);
                        total++;
                    }
                } else {
                    parser.skipChildren();
                }
            }
            require("FeatureCollection".equals(rootType), "Корневой type должен быть FeatureCollection.");
            require(featuresFound, "В файле отсутствует массив features.");
            require(parser.nextToken() == null, "После корневого объекта найдены лишние данные.");
        }

        InspectionResult result = new InspectionResult();
        result.fileSizeBytes = Files.size(path);
        result.totalObjects = total;
        result.typeCounts = types;
        result.restrictionCounts = restrictions;
        result.network = networkBuilder.build();
        return result;
    }

    private static void printResult(String pathText, InspectionResult result) {
        System.out.println("Файл: " + pathText);
        System.out.println("Размер, байт: " + result.fileSizeBytes);
        System.out.println("Всего объектов: " + result.totalObjects);
        System.out.println("\nТипы объектов:");
        result.typeCounts.forEach((type, count) -> System.out.println("  " + type + ": " + count));
        System.out.println("\nТипы ограничений:");
        result.restrictionCounts.forEach((type, count) -> System.out.println("  " + type + ": " + count));
        ExistingNetworkBuilder.NetworkSummary net = result.network;
        System.out.println("\nСуществующая сеть:");
        System.out.println("  источников: " + net.sources + ", участков: " + net.segments
                + ", камер: " + net.chambers + ", максимальная глубина цепочки: " + net.maxDepth);
        net.objectsPerSource.forEach((source, count) ->
                System.out.println("  от источника " + source + " питается объектов: " + count));
        net.warnings.forEach(w -> System.out.println("  Предупреждение: " + w));
    }

    private static void require(boolean condition, String message) throws GeoJsonValidationException {
        if (!condition) {
            throw new GeoJsonValidationException(message);
        }
    }
}