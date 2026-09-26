package ru.heatplanner;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import javax.annotation.PostConstruct;
import javax.annotation.PreDestroy;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.Optional;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Очередь заданий на расчёт. Файл сразу сохраняется на диск (потоково, не в памяти), задание записывается в базу и
 * выполняется фоновым потоком; число одновременных расчётов ограничено (каждый занимает до 2 ГБ памяти), остальные
 * ждут в очереди. После перезапуска сервиса незавершённые задания помечаются как прерванные.
 */
@Service
public class JobService {

    /** Очередь переполнена: расчёт не принят. */
    public static class QueueFullException extends RuntimeException {
        public QueueFullException(String message) {
            super(message);
        }
    }

    private final PlanJobRepository repository;
    private final PlanService planService;
    private final int workers;
    private final int queueLimit;
    private final String storage;
    private ExecutorService pool;
    private Path storageDir;

    public JobService(PlanJobRepository repository, PlanService planService,
                      @Value("${app.calc.workers:1}") int workers,
                      @Value("${app.calc.queue:50}") int queueLimit,
                      @Value("${app.storage.dir:}") String storage) {
        this.repository = repository;
        this.planService = planService;
        this.workers = Math.max(1, workers);
        this.queueLimit = Math.max(1, queueLimit);
        this.storage = storage;
    }

    @PostConstruct
    void start() throws IOException {
        storageDir = storage == null || storage.isEmpty()
                ? Files.createTempDirectory("heat-planner-") : Files.createDirectories(Paths.get(storage));
        pool = Executors.newFixedThreadPool(workers, r -> {
            Thread t = new Thread(r, "plan-job");
            t.setDaemon(true);
            return t;
        });
        for (String id : repository.unfinishedIds()) {
            repository.findById(id).ifPresent(j -> {
                j.setStatus(PlanJob.FAILED);
                j.setMessage("Сервис был перезапущен во время расчёта. Запустите расчёт снова.");
                j.setFinishedAt(Instant.now());
                repository.save(j);
            });
        }
    }

    @PreDestroy
    void stop() {
        pool.shutdownNow();
    }

    public synchronized PlanJob submit(MultipartFile file, MultipartFile roads) throws IOException {
        if (repository.unfinishedIds().size() >= queueLimit) {
            throw new QueueFullException("Очередь расчётов заполнена (" + queueLimit + "). Повторите позже.");
        }
        Path input = Files.createTempFile(storageDir, "upload-", ".geojson");
        file.transferTo(input);
        Path roadsFile = null;
        if (roads != null && !roads.isEmpty()) {
            roadsFile = Files.createTempFile(storageDir, "roads-", ".geojson");
            roads.transferTo(roadsFile);
        }
        PlanJob job = new PlanJob();
        job.setId(UUID.randomUUID().toString());
        job.setStatus(PlanJob.QUEUED);
        job.setFileName(file.getOriginalFilename());
        job.setProgress(0);
        job.setMessage("В очереди");
        job.setCreatedAt(Instant.now());
        repository.save(job);
        final Path roadsPath = roadsFile;
        final String id = job.getId();
        pool.submit(() -> run(id, input, roadsPath));
        return job;
    }

    private void run(String id, Path input, Path roads) {
        try {
            PlanJob job = repository.findById(id).orElseThrow(IllegalStateException::new);
            job.setStatus(PlanJob.RUNNING);
            job.setStartedAt(Instant.now());
            job.setMessage("Расчёт начат");
            repository.save(job);
            PlanService.PlanResult result = planService.compute(input, roads, (p, m) -> progress(id, p, m));
            job = repository.findById(id).orElseThrow(IllegalStateException::new);
            job.setResultJson(result.resultJson);
            job.setExportMain(result.exportMain);
            job.setExportAll(result.exportAll);
            job.setStatus(PlanJob.DONE);
            job.setProgress(100);
            job.setMessage("Готово");
            job.setFinishedAt(Instant.now());
            repository.save(job);
        } catch (Throwable e) {
            repository.findById(id).ifPresent(j -> {
                j.setStatus(PlanJob.FAILED);
                j.setMessage("Не удалось выполнить расчёт: " + e.getMessage());
                j.setFinishedAt(Instant.now());
                repository.save(j);
            });
        } finally {
            try {
                Files.deleteIfExists(input);
                if (roads != null) {
                    Files.deleteIfExists(roads);
                }
            } catch (IOException ignored) {
                // временные файлы будут убраны вместе с каталогом
            }
        }
    }

    private void progress(String id, int percent, String message) {
        repository.findById(id).ifPresent(j -> {
            j.setProgress(percent);
            j.setMessage(message);
            repository.save(j);
        });
    }

    public Optional<PlanJob> find(String id) {
        return repository.findById(id);
    }

    public long queuePosition(PlanJob job) {
        return PlanJob.QUEUED.equals(job.getStatus()) ? repository.queuedBefore(job.getCreatedAt()) + 1 : 0;
    }
}