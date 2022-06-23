# -*- encoding: utf8 -*-
from threading import Thread, Event
import logging

def get_logger(log_level, logdir = None, deploy = False):
    
    logger = logging.getLogger()
    logger.setLevel(log_level)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    if not deploy:
        # Log to console
        handler = logging.StreamHandler()
        handler.setLevel(log_level)
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    # # Log to file
    # if logdir is not None:
    #     ensure_dir(logdir)
    #     log_filepath = os.path.join(logdir, "%s.log" % datetime.datetime.utcnow())
    #     file_handler = logging.FileHandler(log_filepath)
    #     file_handler.setLevel(log_level)
    #     file_handler.setFormatter(formatter)
    #     logger.addHandler(file_handler)
    
    return logger

class ThreadBase(Thread):
    def __init__(self, logger, name):
        super().__init__()
        # self._stop_event = Event()
        self.stopped = False
        self._stopped = False
        self.logger = logger
        self.name = name
        self._print = False

        
    def send_state(self):
        if self._monitoring:
            self._send_state()
        return

    def _send_state(self):
        pass

    def set_monitoring(self, _monitoring):
        self._monitoring = _monitoring
        return 

    def set_print(self, _print):
        self._print = _print
        return
        
    def stop(self):
        self.logger.debug("Stopping %s thread", self.name)
        # self._stop_event.set()
        self.stopped = True


    def stop_base(self):
        self.logger.debug("Stopping %s thread", self.name)
        # self._stop_event.set()
        self._stopped = True

    # def stopped(self):
    #     return self._stop_event.is_set()

    def start(self, daemon):
        self.logger.debug("Starting %s thread", self.name)
        #super().start()
        t = Thread(target=self.run, args=())
        t.daemon = daemon
        t.start()
        
    def run(self):
        self._run()
        
    def run_pokus(self):
        
        while True:
            
            if self._stopped:
                return
                
            try:
                self._run()
            except BaseException as e:
                self.logger.error('THREAD: %s - %s; Restarting thread'%(self.name, e))
            else:
                self.logger.error("THREAD: %s - Exited normally. Restarting"%(self.name))
                
        return
        
    def _run(self):
        pass