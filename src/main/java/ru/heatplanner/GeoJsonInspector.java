package ru.heatplanner;

import com.fasterxml.jackson.core.JsonLocation;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Первый этап обработки: потоковое чтение GeoJSON, проверка структуры и инвентаризация объектов.
 * Проверяет структуру, геометрию, id/тип объектов и обязательные атрибуты по актуальному техническому приложению,
 * а данные существующей сети (диаметры, камеры, связность) передаёт в {@link ExistingNetworkBuilder}.
 * Всё, что не мешает расчёту, но требует внимания, попадает в список замечаний (warnings), а не отклоняет файл.
 */
public class GeoJsonInspector {

    /** Типы ограничений, для которых в таблице 2 приложения есть правило. Остальные типы — расширение, необязательное. */
    static final Set<String> SUPPORTED_RESTRICTIONS = new HashSet<>(java.util.Arrays.asList(
            "oks", "park", "social_area", "prohibited_site", "water", "railway", "road", "tram_tracks",
            "gas_pipeline", "power_cable", "heat_network"));

    /** Типы объектов базового состава (раздел 1.1 приложения). */
    private static final Set<String> BASE_TYPES = new HashSet<>(java.util.Arrays.asList(
            "source", "heat_network", "heat_chamber", "oks_connection_point", "restriction"));

    /** Сколько ошибок геометрии показывать в сообщении (общее число считается полностью). */
    private static final int MAX_REPORTED_GEOMETRY_ERRORS = 20;

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
        /** Сколько объектов каждого object_type. */
        public Map<String, Long> typeCounts;
        /** Сколько ограничений каждого restriction_type. */
        public Map<String, Long> restrictionCounts;
        /** Сводка по существующей сети (см. ExistingNetworkBuilder). */
        public ExistingNetworkBuilder.NetworkSummary network;
        /** Замечания к данным: файл принят, но на что-то стоит обратить внимание (диагностика данных). */
        public List<String> warnings;
    }

    /** Запуск из командной строки или IDE: путь к файлу — единственный аргумент. Нужен для ручных проверок. */
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

    /**
     * Читает файл за один проход и возвращает сводку.
     * Бросает {@link GeoJsonValidationException}, если структура или данные не проходят проверку.
     */
    public static InspectionResult inspect(Path path) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        // Повторяющийся ключ внутри одного JSON-объекта — ошибка, а не «победит последний».
        mapper.enable(JsonParser.Feature.STRICT_DUPLICATE_DETECTION);
        Map<String, Long> types = new TreeMap<>();
        Map<String, Long> restrictions = new TreeMap<>();
        Set<String> seenIds = new HashSet<>();
        // Получает объекты существующей сети по ходу чтения; результат забираем в конце.
        ExistingNetworkBuilder networkBuilder = new ExistingNetworkBuilder();
        // Ошибки геометрии копим по всему файлу, чтобы показать их разом, а не по одной за загрузку.
        List<String> geometryErrors = new ArrayList<>();
        long geometryErrorCount = 0;
        long total = 0;
        long zeroFlowPoints = 0;
        boolean featuresFound = false;
        String rootType = null;

        try (JsonParser parser = mapper.getFactory().createParser(path.toFile())) {
            // Обходим корневой объект по полям: type, features, а всё остальное (name, crs) пропускаем.
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
                        // Потоковое чтение: в памяти одновременно только один Feature, а не весь файл (файлы до 3 ГБ).
                        JsonNode feature = mapper.readTree(parser);
                        require("Feature".equals(feature.path("type").asText()),
                                "Неверный type у Feature #" + (total + 1));

                        JsonNode geometry = feature.path("geometry");
                        require(geometry.isObject(), "Нет geometry у Feature #" + (total + 1));

                        GeoJsonGeometryValidator.validate(geometry, "Feature #" + (total + 1));

                        JsonNode properties = feature.path("properties");
                        require(properties.isObject(), "Нет properties у Feature #" + (total + 1));

                        // id может быть строкой или числом; строка "1" и число 1 — разные идентификаторы,
                        // поэтому тип входит в ключ проверки уникальности (формат id не интерпретируется).
                        JsonNode idNode = properties.path("id");
                        require(!idNode.isMissingNode() && !idNode.isNull()
                                        && (idNode.isTextual() || idNode.isNumber())
                                        && !idNode.asText().isBlank(),
                                "Нет id у Feature #" + (total + 1));
                        String id = idNode.asText();
                        require(seenIds.add((idNode.isTextual() ? "s:" : "n:") + id), "Повторяющийся id \"" + id + "\" у Feature #" + (total + 1));

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
                        // Каждая точка подключения — самостоятельная цель: расход обязателен (раздел 1.1).
                        if ("oks_connection_point".equals(type.asText())) {
                            JsonNode flow = properties.path("flow_tph");
                            require(flow.isNumber() && flow.asDouble() >= 0,
                                    "oks_connection_point \"" + id + "\" (Feature #" + (total + 1)
                                            + "): flow_tph должен быть неотрицательным числом.");
                            if (flow.asDouble() == 0) {
                                zeroFlowPoints++;
                            }
                        }
                        // Форма и диапазон координат уже проверены GeoJsonGeometryValidator; здесь — соответствие типа геометрии типу объекта.
                        String geometryProblem = GeometryChecker.checkKind(type.asText(), geometry.path("type").asText());
                        if (geometryProblem != null) {
                            geometryErrorCount++;
                            if (geometryErrors.size() < MAX_REPORTED_GEOMETRY_ERRORS) {
                                geometryErrors.add(type.asText() + " \"" + id + "\" (Feature #" + (total + 1) + "): "
                                        + geometryProblem + ".");
                            }
                        } else {
                            // source, heat_network и heat_chamber уходят в разбор существующей сети, остальные типы билдер игнорирует.
                            networkBuilder.add(feature, type.asText(), id, total + 1);
                        }
                        total++;
                    }
                } else {
                    parser.skipChildren();
                }
            }
            require("FeatureCollection".equals(rootType), "Корневой type должен быть FeatureCollection.");
            require(featuresFound, "В файле отсутствует массив features.");
            require(parser.nextToken() == null, "После корневого объекта найдены лишние данные.");
        } catch (JsonProcessingException e) {
            // Синтаксически неверный JSON (обрыв файла, лишняя запятая, повторяющийся ключ, не тот символ).
            // Раньше это исключение не ловилось, и пользователь получал ошибку сервера 500 вместо объяснения.
            throw new GeoJsonValidationException("Файл не является корректным JSON" + describeLocation(e.getLocation())
                    + ". Подробности: " + e.getOriginalMessage() + ".");
        }

        if (geometryErrorCount > 0) {
            StringBuilder text = new StringBuilder("Ошибки геометрии (" + geometryErrorCount + "):");
            geometryErrors.forEach(problem -> text.append("\n- ").append(problem));
            if (geometryErrorCount > geometryErrors.size()) {
                text.append("\n... и ещё ").append(geometryErrorCount - geometryErrors.size());
            }
            throw new GeoJsonValidationException(text.toString());
        }

        InspectionResult result = new InspectionResult();
        result.fileSizeBytes = Files.size(path);
        result.totalObjects = total;
        result.typeCounts = types;
        result.restrictionCounts = restrictions;
        // Файл прочитан целиком — теперь можно проверить существующую сеть (камеры, связность, диаметры).
        result.network = networkBuilder.build();
        result.warnings = new ArrayList<>(inputWarnings(types, restrictions, zeroFlowPoints));
        result.warnings.addAll(result.network.warnings);
        return result;
    }

    /** Замечания к составу данных: что в файле не входит в базовый состав и не будет использовано. */
    private static List<String> inputWarnings(Map<String, Long> types, Map<String, Long> restrictions, long zeroFlowPoints) {
        List<String> out = new ArrayList<>();
        if (!types.containsKey("oks_connection_point")) {
            out.add("В файле нет точек подключения ОКС (oks_connection_point): подключать нечего.");
        }
        for (Map.Entry<String, Long> e : types.entrySet()) {
            if (BASE_TYPES.contains(e.getKey())) {
                continue;
            }
            boolean building = "oks_existing".equals(e.getKey()) || "oks_future".equals(e.getKey());
            out.add("Объекты типа " + e.getKey() + " (" + e.getValue() + " шт.) не входят в базовый состав и в расчёте не используются"
                    + (building ? ": здания передаются как ограничения restriction с типом oks." : "."));
        }
        List<String> unsupported = new ArrayList<>();
        for (Map.Entry<String, Long> e : restrictions.entrySet()) {
            if (!SUPPORTED_RESTRICTIONS.contains(e.getKey())) {
                unsupported.add(e.getKey() + " (" + e.getValue() + ")");
            }
        }
        if (!unsupported.isEmpty()) {
            out.add("Типы ограничений, которых нет в таблице 2 приложения (поддержка необязательна): "
                    + String.join(", ", unsupported) + ".");
        }
        if (zeroFlowPoints > 0) {
            out.add("У точек подключения нулевой расход (flow_tph = 0): " + zeroFlowPoints + " шт.");
        }
        return out;
    }

    /** « (строка N, столбец M)» для сообщения об ошибке JSON; пустая строка, если положение неизвестно. */
    private static String describeLocation(JsonLocation location) {
        if (location == null || location.getLineNr() < 1) {
            return "";
        }
        return " (строка " + location.getLineNr() + ", столбец " + location.getColumnNr() + ")";
    }

    /** Печать сводки в консоль — только для запуска через main. */
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
        System.out.println("  источников: " + net.sources + ", участков: " + net.segments + ", камер: " + net.chambers);
        result.warnings.forEach(w -> System.out.println("  Замечание: " + w));
    }

    /** Короткая проверка условия: если не выполнено — исключение с понятным сообщением. */
    private static void require(boolean condition, String message) throws GeoJsonValidationException {
        if (!condition) {
            throw new GeoJsonValidationException(message);
        }
    }
}