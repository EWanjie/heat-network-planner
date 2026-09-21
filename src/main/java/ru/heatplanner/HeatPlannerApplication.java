package ru.heatplanner;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

/** Точка запуска сервиса: поднимает встроенный веб-сервер (порт 8080) со страницей и API. */
@SpringBootApplication
public class HeatPlannerApplication {

    public static void main(String[] args) {
        SpringApplication.run(HeatPlannerApplication.class, args);
    }
}