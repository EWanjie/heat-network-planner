# Развёртывание

## Docker (основной способ, docker-compose 1.29.2)
```
docker-compose up --build
```
Поднимаются PostgreSQL 16 и приложение (образ собирается из `Dockerfile`: Maven на Java 11, среда исполнения Java 11). Страница и API
доступны на порту 8080, документация API — `http://localhost:8080/swagger-ui.html`. Память Java задаётся `JAVA_OPTS` в
`docker-compose.yml` (по умолчанию `-Xmx6g` при 16 ГБ ОЗУ). Данные базы хранятся в томе `pgdata`, загруженные файлы — в томе `uploads`.
Целевая ОС — Ubuntu Server 22.

## Локальный запуск без Docker
```
mvn clean package
java -jar target/heat-network-planner-1.0-SNAPSHOT.jar
```
База — файл H2 в `./data/planner`. Порт: `--server.port=8090`. Настройки очереди: `app.calc.workers`, `app.calc.queue`, `app.storage.dir`.

## API
1. `POST /api/upload` — проверка файла, сводка и диагностика данных.
2. `POST /api/plan` (поля `file`, необязательно `roads`) — постановка расчёта, ответ `202 {jobId}`.
3. `GET /api/plan/{jobId}` — состояние (`QUEUED`, `RUNNING`, `DONE`, `FAILED`), прогресс и позиция в очереди; для `DONE` — результат.
4. `GET /api/plan/{jobId}/export?additional=false` — GeoJSON по разделу 7.

Задания и результаты хранятся в базе; после перезапуска сервиса незавершённые задания помечаются как прерванные.