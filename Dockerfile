# Сборка и запуск сервиса: docker-compose up --build
# Этап 1: сборка jar (тесты запускаются отдельно командой mvn test).
FROM maven:3.8.8-eclipse-temurin-11 AS build
WORKDIR /build
COPY pom.xml .
RUN mvn -q -B dependency:go-offline
COPY src ./src
RUN mvn -q -B package -DskipTests

# Этап 2: среда исполнения Java 11.
FROM eclipse-temurin:11-jre
WORKDIR /app
COPY --from=build /build/target/heat-network-planner-1.0-SNAPSHOT.jar app.jar
ENV JAVA_OPTS="-Xmx6g"
EXPOSE 8080
ENTRYPOINT ["sh", "-c", "exec java $JAVA_OPTS -jar app.jar"]