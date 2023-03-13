from threading import Thread
from queue import Full, Queue
from era_5g_interface.task_handler_gstreamer import TaskHandlerGstreamer
from era_5g_object_detection_distributed_interface.task_handler_distributed import TaskHandlerDistributed
from era_5g_object_detection_distributed_worker import detector_task


class TaskHandlerGstreamerDistributed(TaskHandlerGstreamer, TaskHandlerDistributed):
    """
    Task handler which stores the retrieved images to the RabbitMQ broker.
    """
    def __init__(self, sid: str, port: int, jobs_info_queue: Queue, **kw) -> None:
        super().__init__(sid, port, jobs_info_queue, **kw)

   