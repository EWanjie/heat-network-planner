package ru.heatplanner;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

import java.time.Instant;
import java.util.List;

public interface PlanJobRepository extends JpaRepository<PlanJob, String> {

    @Query("select j.id from PlanJob j where j.status in ('QUEUED', 'RUNNING')")
    List<String> unfinishedIds();

    /** Сколько заданий стоят в очереди раньше данного. */
    @Query("select count(j) from PlanJob j where j.status = 'QUEUED' and j.createdAt < :before")
    long queuedBefore(@Param("before") Instant before);
}