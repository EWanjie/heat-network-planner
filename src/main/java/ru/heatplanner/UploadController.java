package ru.heatplanner;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

@RestController
public class UploadController {

    @PostMapping("/api/upload")
    public ResponseEntity<?> upload(@RequestParam("file") MultipartFile file) throws IOException {
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
        } finally {
            Files.deleteIfExists(tempFile);
        }
    }
}