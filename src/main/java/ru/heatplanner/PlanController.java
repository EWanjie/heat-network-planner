package ru.heatplanner;

import com.fasterxml.jackson.databind.node.ObjectNode;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

/** Расчёт вариантов подключения: тот же файл, что и при загрузке, но с полным расчётом. */
@RestController
public class PlanController {

    /**
     * POST /api/plan, поле формы "file".
     * 200 — JSON: geojson (все варианты по разделу 10 ТЗ) и meta (пояснения); 400 — файл не прошёл проверку;
     * 422 — файл корректен, но расчёт невозможен (нет источника, сети или ОКС).
     */
    @PostMapping("/api/plan")
    public ResponseEntity<?> plan(@RequestParam("file") MultipartFile file) throws IOException {
        if (file.isEmpty()) {
            return ResponseEntity.badRequest().body("Файл пуст.");
        }
        Path tempFile = Files.createTempFile("plan-", ".geojson");
        try {
            file.transferTo(tempFile);
            GeoJsonInspector.InspectionResult checked = GeoJsonInspector.inspect(tempFile);
            PlanModel model = PlanModel.parse(tempFile, checked.upstreamLinks);
            Planner.Result result = Planner.solve(model);
            ObjectNode body = PlanWriter.write(result);
            return ResponseEntity.ok(body);
        } catch (GeoJsonInspector.GeoJsonValidationException e) {
            return ResponseEntity.badRequest().body("Файл не прошёл проверку: " + e.getMessage());
        } catch (IllegalArgumentException e) {
            return ResponseEntity.unprocessableEntity().body(e.getMessage());
        } catch (InterruptedException | java.util.concurrent.ExecutionException e) {
            return ResponseEntity.status(500).body("Расчёт прерван: " + e.getMessage());
        } finally {
            Files.deleteIfExists(tempFile);
        }
    }

    /** GET /api/rules — правила расчёта из технического приложения как данные (для экрана «Как это посчитано»). */
    @GetMapping("/api/rules")
    public ObjectNode rules() {
        return PlanWriter.rules();
    }
}
