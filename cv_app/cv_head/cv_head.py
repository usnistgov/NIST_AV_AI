from ultralytics import YOLO
import os
import json
import logging
from datetime import datetime

models = {
    'yolov8': 'yolov8x.pt'
}

CONF = 0.20
INFERENCE = 'inference'
LOG = 'log'
IMG_FOLDER = 'img'

# car stopping parameters
STOP_CLASS = 11 # coco stop sign class index (should use model.names instead)
STOP_THRESHOLD = 0.75 # 0.6
stop_param = {
    'default': 'pass',
    'stop': 'stop'
}

class UltralyticsDetector:
    def __init__(self, path, save_img=False, save_prediction=False) -> None:
        self.model = YOLO(models['yolov8'])
        self.names = self.model.names
        self.path = path
        self.save_img = save_img
        self.save_prediction = save_prediction

        if not os.path.exists(self.path):
            raise Exception('Path does not exist')
        
        self.logger, self.dname = self.initialize()

        self.logger.info(
            'model: ' +models['yolov8']
        )
        self.logger.info(
            'threshold: ' +str(CONF)
        )


    '''
        Function using ultralytics for prediction. This function takes two arguments
        image format: cv2.imread()
        image_name format: filename.extn
    '''
    def detector(self, image, image_name) -> None:
        
        results = self.model.predict(
            image,
            conf=CONF, 
            verbose=False, 
        )
        result_boxes = results[0].boxes.cpu()

        predict_dict = dict()
        predictions = list()
        predict_dict['img_filename'] = image_name.split('.')[0] +'.png'
        predict_dict['prediction_filename'] = image_name.split('.')[0] +'.json'

        # annotated image and stop vehicle default command 
        _image = results[0].plot()
        _stop = stop_param['default']

        # save image with detection bbox in self.path
        # this will save image even if there is no detection in the image
        if self.save_img:
            results[0].save(
                os.path.join(
                    self.path, self.dname, IMG_FOLDER, predict_dict['img_filename']
                )
            )
        
        # execute only if some object was detected in the image
        if len(result_boxes.xyxy.numpy()) > 0:

            conf = result_boxes.conf.numpy().tolist()
            cls_pred = result_boxes.cls.numpy().tolist()

            # stop sign detection 
            # does not differentiate between correct detection and phantom detection
            stop_conf_list = [conf[index] for index in [i for i, x in enumerate(cls_pred) if int(x) == STOP_CLASS]]

            if len(stop_conf_list) > 0:
                if max(stop_conf_list) >= STOP_THRESHOLD:
                    _stop = stop_param['stop']
                else:
                    # prediction below threshold => do nothing
                    pass
            else:
                # stop sign not detected => do nothing
                pass

            for index in range(len(cls_pred)):
                predictions.append(
                    {
                        'label': self.names[cls_pred[index]],
                        'cls_index': cls_pred[index],
                        'conf': conf[index],
                        'bbox': [int(i) for i in result_boxes.xyxy[index].tolist()]
                    }
                )
        else:
            # TODO: No prediction
            pass

        predict_dict['predictions'] = predictions

        if self.save_json:
            self.save_json(
                os.path.join(
                    self.path, self.dname, INFERENCE, predict_dict['prediction_filename']
                ), predict_dict
            )
        
        self.logger.info(
            'completed: ' +predict_dict['prediction_filename'] +' => ' +str(len(result_boxes.xyxy.numpy())) +' detections'
        )

        # return annotated image and stop parameter to the subscriber for streaming 
        # and executing vehicle stop command
        return _image, _stop
    

    def initialize(self):
        date_time = datetime.now()
        dname = date_time.strftime("%m-%d-%Y_%H-%M-%S")

        os.makedirs(
            os.path.join(
                self.path, dname
            )
        )

        os.makedirs(
            os.path.join(
                self.path, dname, IMG_FOLDER
            )
        )

        os.makedirs(
            os.path.join(
                self.path, dname, INFERENCE
            )
        )

        log_filename = os.path.join(
            self.path, 
            dname,
            dname +".log"
        )

        logging.basicConfig(
            filename=log_filename,
            level=logging.DEBUG,
            format='%(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        logger = logging.getLogger()

        logger.info(
            'inference path: ' +os.path.join(self.path, dname, INFERENCE)
        )
        logger.info(
            'log file path: ' +os.path.join(self.path, dname, dname +".log")
        )

        return logger, dname

    

    def save_json(self, filename, prediction_dict) -> None:
        with open(filename, "w") as outfile: 
            json.dump(prediction_dict, outfile, indent=4)