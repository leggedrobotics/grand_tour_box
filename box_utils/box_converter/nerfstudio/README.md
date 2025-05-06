
- build the container
    ```
    DOCKER_BUILDKIT=1 docker compose build
    ```

- run the container
    ```
    docker compose up -d && docker exec -it nerfstudio bash
    ```