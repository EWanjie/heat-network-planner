package ru.heatplanner;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.tags.Tag;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;

/**
 * Расчёт трасс подключения. Расчёт занимает минуты, поэтому он асинхронный: POST принимает файл и возвращает номер
 * задания, состояние и результат читаются по номеру, выгрузка — отдельным запросом.
 */
@RestController
@Tag(name = "Расчёт трасс", description = "Постановка расчёта, состояние задания, результат и выгрузка GeoJSON по разделу 7 приложения")
public class PlanController {

    private final JobService jobs;
    private final ObjectMapper mapper = new ObjectMapper();

    public PlanController(JobService jobs) {
        this.jobs = jobs;
    }

    @Operation(summary = "Поставить расчёт в очередь",
            description = "Принимает GeoJSON (поле file) и, необязательно, дороги OpenStreetMap (поле roads). "
                    + "Возвращает номер задания; результат читается через GET /api/plan/{jobId}.")
    @PostMapping(value = "/api/plan", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    public ResponseEntity<?> submit(@RequestParam("file") MultipartFile file,
                                    @RequestParam(value = "roads", required = false) MultipartFile roads) throws IOException {
        if (file.isEmpty()) {
            return ResponseEntity.badRequest().body("Файл пуст.");
        }
        try {
            PlanJob job = jobs.submit(file, roads);
            ObjectNode body = mapper.createObjectNode();
            body.put("jobId", job.getId());
            body.put("status", job.getStatus());
            return ResponseEntity.accepted().body(body);
        } catch (JobService.QueueFullException e) {
            return ResponseEntity.status(429).body(e.getMessage());
        }
    }

    @Operation(summary = "Состояние задания и результат",
            description = "QUEUED, RUNNING, DONE или FAILED; для DONE в поле result лежат варианты, проверка и диагностика.")
    @GetMapping("/api/plan/{jobId}")
    public ResponseEntity<?> status(@Parameter(description = "Номер задания") @PathVariable String jobId) throws IOException {
        return jobs.find(jobId).<ResponseEntity<?>>map(job -> {
            try {
                ObjectNode body = mapper.createObjectNode();
                body.put("jobId", job.getId());
                body.put("status", job.getStatus());
                body.put("progress", job.getProgress());
                body.put("message", job.getMessage());
                body.put("fileName", job.getFileName());
                body.put("queuePosition", jobs.queuePosition(job));
                if (PlanJob.DONE.equals(job.getStatus()) && job.getResultJson() != null) {
                    body.set("result", mapper.readTree(job.getResultJson()));
                }
                return ResponseEntity.ok(body);
            } catch (IOException e) {
                return ResponseEntity.status(500).body("Не удалось прочитать результат: " + e.getMessage());
            }
        }).orElseGet(() -> ResponseEntity.status(404).body("Задание не найдено."));
    }

    @Operation(summary = "Выгрузка результата в GeoJSON",
            description = "Один FeatureCollection в WGS 84 по разделу 7 приложения; варианты различаются по variant_id. "
                    + "additional=true добавляет дополнительные варианты (a1, a2, ...).")
    @GetMapping("/api/plan/{jobId}/export")
    public ResponseEntity<?> export(@PathVariable String jobId,
                                    @RequestParam(value = "additional", defaultValue = "false") boolean additional) {
        return jobs.find(jobId).<ResponseEntity<?>>map(job -> {
            String body = additional ? job.getExportAll() : job.getExportMain();
            if (!PlanJob.DONE.equals(job.getStatus()) || body == null) {
                return ResponseEntity.status(409).body("Расчёт ещё не завершён.");
            }
            return ResponseEntity.ok()
                    .header("Content-Disposition", "attachment; filename=\"variants_2d.geojson\"")
                    .contentType(MediaType.parseMediaType("application/geo+json"))
                    .body(body);
        }).orElseGet(() -> ResponseEntity.status(404).body("Задание не найдено."));
    }
}