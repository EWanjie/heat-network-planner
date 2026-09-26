package ru.heatplanner;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.Id;
import javax.persistence.Table;
import java.time.Instant;

/** Задание на расчёт: состояние, прогресс и результат хранятся в базе данных (PostgreSQL в развёртывании, H2 локально). */
@Entity
@Table(name = "plan_job")
public class PlanJob {

    public static final String QUEUED = "QUEUED";
    public static final String RUNNING = "RUNNING";
    public static final String DONE = "DONE";
    public static final String FAILED = "FAILED";

    @Id
    private String id;
    @Column(nullable = false)
    private String status;
    private String fileName;
    private int progress;
    @Column(length = 2000)
    private String message;
    private Instant createdAt;
    private Instant startedAt;
    private Instant finishedAt;
    @Column(columnDefinition = "text")
    private String resultJson;
    @Column(columnDefinition = "text")
    private String exportMain;
    @Column(columnDefinition = "text")
    private String exportAll;

    public String getId() { return id; }
    public void setId(String id) { this.id = id; }
    public String getStatus() { return status; }
    public void setStatus(String status) { this.status = status; }
    public String getFileName() { return fileName; }
    public void setFileName(String fileName) { this.fileName = fileName; }
    public int getProgress() { return progress; }
    public void setProgress(int progress) { this.progress = progress; }
    public String getMessage() { return message; }
    public void setMessage(String message) { this.message = message; }
    public Instant getCreatedAt() { return createdAt; }
    public void setCreatedAt(Instant createdAt) { this.createdAt = createdAt; }
    public Instant getStartedAt() { return startedAt; }
    public void setStartedAt(Instant startedAt) { this.startedAt = startedAt; }
    public Instant getFinishedAt() { return finishedAt; }
    public void setFinishedAt(Instant finishedAt) { this.finishedAt = finishedAt; }
    public String getResultJson() { return resultJson; }
    public void setResultJson(String resultJson) { this.resultJson = resultJson; }
    public String getExportMain() { return exportMain; }
    public void setExportMain(String exportMain) { this.exportMain = exportMain; }
    public String getExportAll() { return exportAll; }
    public void setExportAll(String exportAll) { this.exportAll = exportAll; }
}