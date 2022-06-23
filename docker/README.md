# Dockerfiles and related stuff

Contains dockerfiles and related running scripts.


## Files

### Dockerfile

Dockerfile for all-in-one CSS (control service server). Currently uses ros:galactic-ros-base image and runs the runs.sh script (described bellow).


### Dockerfile.distributed-css

Dockerfile for distributed variant of CSS (control service server). Currently uses ros:galactic-ros-base image and runs the test_celery.sh script (described in celery_example folder).

### Dockerfile.ml-worker

Dockerfile for celery worker. Currently uses the python:3.8 docker image, with installed celery and redis. Runs the run_celery.sh script

### run.sh

Runs the all-in-one CSS node.

### run_celery.sh

Runs the celery task.

## Build

Alle images could be build from the main folder (ML_Toolboxes) using following commands:

```bash
sudo docker build . -f docker/Dockerfile -t but5gera/ros-ml_service-standalone:latest
sudo docker build . -f docker/Dockerfile.distributed-css -t but5gera/ros-ml_service-distributed_interface:latest
sudo docker build . -f docker/Dockerfile.ml-worker -t but5gera/ros-ml_service-distributed_worker:latest
```

