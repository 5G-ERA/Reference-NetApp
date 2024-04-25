import base64
import os
from queue import Queue

import cv2
import pycocotools.mask as masks_util
import numpy as np

from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS
from era_5g_object_detection_standalone.worker import BATCH_SIZE
from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker

image_queue = Queue(maxsize=BATCH_SIZE + 1)

try:
    TEST_VIDEO_FILE = os.environ["TEST_VIDEO_FILE"]
except KeyError as e:
    raise Exception(f"Failed to run example, env variable {e} not set.")

if not os.path.isfile(TEST_VIDEO_FILE):
    raise Exception("TEST_VIDEO_FILE does not contain valid path to a file.")

DEBUG_PRINT_SCORE = False  # useful for FPS detector
DEBUG_PRINT_DELAY = False  # prints the delay between capturing image and receiving the results
DEBUG_DRAW_MASKS = True  # draw segmentation masks (if provided by detector)


class ObjectDetector(MMDetectorWorker):
    def __init__(self, image_queue: Queue, **kw):
        super().__init__(image_queue, self.publish_results, **kw)

    def publish_results(self, results, metadata):
        
        detections = self.prepare_detections_for_publishing(results)

        print(detections)


def main() -> None:
    detector1 = ObjectDetector(image_queue)
    # detector1.start()

    cap = cv2.VideoCapture(TEST_VIDEO_FILE)
    if not cap.isOpened():
        raise Exception("Cannot open video file")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        results = detector1.process_image(frame)
        print(results)
        for d in results:
            score = float(d[1])
            if DEBUG_PRINT_SCORE and score > 0:
                print(score)
            cls_name = d[3]
            # Draw detection into frame.
            x1, y1, x2, y2 = [int(coord) for coord in d[0]]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                frame,
                f"{cls_name} ({score * 100:.0f})%",
                (x1, y1 - 5),
                font,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

            if DEBUG_DRAW_MASKS and len(d) > 4:
                encoded_mask = d[4]
                encoded_mask["counts"] = base64.b64decode(encoded_mask["counts"])  # Base64 decode
                mask = masks_util.decode(encoded_mask).astype(bool)  # RLE decode
                color_mask = np.random.randint(0, 256, (1, 3), dtype=np.uint8)
                frame[mask] = frame[mask] * 0.5 + color_mask * 0.5

        try:
            cv2.imshow("Results", frame)
            cv2.waitKey(1)
        except Exception as ex:
            print(repr(ex))


if __name__ == "__main__":
    main()
