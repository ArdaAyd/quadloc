# Konteyner

```bash
podman build -t quadloc -f docker/Dockerfile .
podman run --rm -it --device nvidia.com/gpu=all -v $PWD:/ws/quadloc -w /ws/quadloc quadloc
```

Docker ile aynı komutlar çalışır; GPU bayrağı `--gpus all` olur.

GPU olmadan (yazılımsal render, yavaş):

```bash
podman run --rm -it -e LIBGL_ALWAYS_SOFTWARE=1 -v $PWD:/ws/quadloc -w /ws/quadloc quadloc
```

İmaj, ROS paketlerini `snapshots.ros.org/jazzy/2026-10-05` deposundan
kurar ve taban imajı digest ile sabitler; böylece her inşada aynı sürümler
gelir.
