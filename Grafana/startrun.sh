docker build -t my-grafana .
docker run -d -p 3000:3000 \
  -v grafana-storage:/var/lib/grafana \
  --name=grafana \
  my-grafana