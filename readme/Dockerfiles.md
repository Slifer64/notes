# Multi-stage docker images I use for developing with VSCode

https://github.com/athackst/dockerfiles/tree/main


## `ENTRYPOINT`

### Syntax

```docker
ENTRYPOINT command param1 param2
#example
ENTRYPOINT python app.py
```
or
```docker
ENTRYPOINT ["executable", "param1", "param2"]
#example
ENTRYPOINT ["python", "app.py"]
```

### ENTRYPOINT With CMD

`CMD` provides the default arguments for an executing container.
Unlike using `ENTRYPOINT` alone, you can use `CMD` to define default arguments for the `ENTRYPOINT` instruction. 
For example:
```docker
ENTRYPOINT ["python", "app.py"]
CMD ["--help"]
```
Starting a Docker container without any command line arguments means `python app.py --help` will execute by default. However, providing arguments, e.g. `docker run <image> --version`, will replace the default `CMD` arguments, resulting in `python app.py --version`.

### Override ENTRYPOINT

E.g., your image has a Python script as its ENTRYPOINT, but you want to open a shell inside the container instead:
```bash
docker run --entrypoint <image> “/bin/bash”
```

### Remarks

- Ensure ENTRYPOINT Scripts Are Executable and Properly Formatted, i.e.:
    ```docker
    COPY entrypoint.sh /entrypoint.sh
    RUN chmod +x /entrypoint.sh
    ENTRYPOINT ["/entrypoint.sh"]
    ```

### GPU

With `docker compose`:
```yaml
services:
  gpu:
    image: ${DOCKER_IMAGE}
    container_name: ${container_name}
    privileged: true
    network_mode: host
    command: /bin/bash
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - $XAUTHORITY:/root/.Xauthority
    environment:
      QT_X11_NO_MITSHM: 1
      DISPLAY: $DISPLAY
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: all
```
With `docker run`:
```bash
docker run ... --gpus=all
```