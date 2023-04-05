import logging
import time


class RateTimer:
    """ Iteration timer used to control the speed of a loop.    

    The timer takes into account the time consumed by processed work 
    and sleeps only for the time remaining to complete the iteration.
    Code is loosely inpired by ROS Rate object and by
    https://stackoverflow.com/questions/23252796/control-the-speed-of-a-loop/23253617
    """

    def __init__(self, rate=None, iteration_time=None, name="timer",
                 time_function=time.time, iteration_miss_warning=False,
                 interval_reinit_after_miss=True, verbose=False):
        """ Initialize RateTimer.

        Args:
            rate (int, optional): Required number of iterations per 
            second (i.e. rate in Hz, or FPS). 
                Note: Either rate or iteration_time has to be provided.
            iteration_time (float, optional): Required length of one 
                interation in seconds.
                Note: Either rate or iteration_time has to be provided.
            name (str, optional): User-defined name of the RateTimer 
                object. Default is 'timer'.
            time_function (callable): Function that is called in order
                to determine current time. Default is time.time.
            iteration_miss_warning (bool): Logging flag. If True, each
                iteration misss is logged as a warning. Default is 
                False.
            interval_reinit_after_miss (bool): Default is True.
                Flag to determine if a new interation interval should 
                start after iteration miss (when set to True) or if the
                next iteration should be attempted as if there was no 
                miss (when set to False).
            verbose (bool): Logging verbosity flag. Default is False.
        """
        
        if rate is None and iteration_time is None:
            raise RuntimeError("Either 'rate' or 'iteration_time' has to be specfied for RateTimer.")
        if rate is not None and iteration_time is not None:
            raise RuntimeError("Only 'rate' or 'interation_time' can be provided for RateTimer, but not both.")

        # Either rate in Hz (FPS) or iteration time (in seconds) can be given
        if iteration_time is not None:
            self.total_iteration_time = iteration_time
            self.rate = 1.0 / iteration_time
        else:
            self.total_iteration_time = 1.0 / rate
            self.rate = rate

        # time.time is chosen as default time_function,
        # but time.perf_counter could probably also be used
        self.time_function = time_function

        self.name = name
        self.iteration_miss_warning = iteration_miss_warning
        self.interval_reinit_after_miss = interval_reinit_after_miss
        self.verbose = verbose

        self.missed_iterations = 0
        self.times_called = 0
        self.creation_time = self.time_function()
        self.next_iteration_time = self.creation_time + self.total_iteration_time

        if self.verbose:
            logging.info(f"RateTimer ({self.name}) created.")

    def sleep(self):
        self.times_called += 1

        current_time = self.time_function()
        if current_time > self.next_iteration_time:
            # Processing took longer than defined iteration time. Do not sleep.
            self.missed_iterations += 1
            
            if self.interval_reinit_after_miss:
                # Plan new iterations starting from this time point (default behaviour)
                self.next_iteration_time = current_time + self.total_iteration_time  
            else:
                # Another possibility is to try to keep the old interval between the iterations as before miss 
                # (this may be problematic in case of larger time miss, because more iterations may then happen without sleep)
                self.next_iteration_time += self.total_iteration_time  
            
            miss_msg = f"RateTimer ({self.name}): Iteration missed."
            if self.iteration_miss_warning: 
                logging.warning(miss_msg)
            else:
                if self.verbose:
                    logging.info(miss_msg)

            # Do not perform any sleep
            return

        # time remaining to next iteration
        sleep_time = self.next_iteration_time - self.time_function()
        if sleep_time < 0:
            # sleep time must not be negative (it can happen due to threading)
            sleep_time = 0
        self.next_iteration_time += self.total_iteration_time
        time.sleep(sleep_time)

    def get_statistics(self):
        data = {"name": self.name, 
                "rate": self.rate,
                "iteration_time": self.total_iteration_time,
                "times_called": self.times_called, 
                "missed_iterations": self.missed_iterations,
                }
        return data


def rate_timer_example():
    """Example usage of RateTimer."""

    import random

    logging.getLogger().setLevel(logging.INFO)

    rate = 2  # FPS
    iteration_time = 1.0 / rate
    logging.info(f"Using rate timer with rate {rate} Hz ({iteration_time} s).")

    # RateTimer should be created right before the loop starts
    rate_timer = RateTimer(rate, verbose=True, iteration_miss_warning=True)

    while True:
        logging.info(f"Time is: {time.time()}")
        
        # Generate random sleep time which is sometimes too long and forces iteration miss
        rand_time = random.uniform(iteration_time*0.5, iteration_time*1.1)
        time.sleep(rand_time) # simulate work

        # Sleep until next interation
        rate_timer.sleep()


if __name__ == "__main__":
    rate_timer_example()
