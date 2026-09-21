package ru.heatplanner;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.springframework.util.MultiValueMap;
import java.util.List;
import com.fasterxml.jackson.core.JsonProcessingException;

/** REST-вход сервиса: принимает GeoJSON от страницы и возвращает результат проверки. */
@RestController
public class UploadController {

    /**
     * POST /api/upload, поле формы "file".
     * 200 — JSON со сводкой (InspectionResult); 400 — текст причины, если файл пуст или не прошёл проверку.
     */
    @PostMapping("/api/upload")
    public ResponseEntity<?> upload(@RequestParam MultiValueMap<String, MultipartFile> uploads) throws IOException {
        MultipartFile file = uploads.getFirst("file");
        if (uploads.values().stream().mapToLong(List::size).sum() != 1 || file == null) {
            return ResponseEntity.badRequest().body("Выберите ровно один файл.");
        }
        if (file.isEmpty()) {
            return ResponseEntity.badRequest().body("Файл пуст.");
        }

        // Пишем во временный файл на диске — не держим загрузку в памяти целиком.
        Path tempFile = Files.createTempFile("upload-", ".geojson");
        try {
            file.transferTo(tempFile);
            GeoJsonInspector.InspectionResult result = GeoJsonInspector.inspect(tempFile);
            return ResponseEntity.ok(result);
        } catch (GeoJsonInspector.GeoJsonValidationException e) {
            return ResponseEntity.badRequest().body("Файл не прошёл проверку: " + e.getMessage());
        } catch (JsonProcessingException e) {
            return ResponseEntity.badRequest().body("Не удалось прочитать JSON. Проверьте содержимое файла.");
        } finally {
            Files.deleteIfExists(tempFile);
        }
    }
}
