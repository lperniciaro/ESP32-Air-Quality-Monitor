# Build and run
docker build -t my-influxdb .
docker run -d -p 8086:8086 --name influxdb my-influxdb

