import threading
import queue
import worker
import subprocess
import numpy as np


class MyThread(threading.Thread):
    def __init__(self, target, args):
        super().__init__(target=target, args=args)
        self.result = None

    def run(self):
        self.result = self._target(*self._args, **self._kwargs)

class run_simulation():
    def __init__(self):
        self.starting_object_id = 0
    
    def find_first_zero(self):
        print(self.starting_object_id)
        for index, val in np.ndenumerate(self.data_generation_log):
            if index[0] >= self.starting_object_id:
                if val == 0:
                    return index[0]  # Return the index of the first 0 value
                    
        return -1  # Return -1 if 0 is not found in the array

    # the start of the object id is decided based on data generation of how many objects has been completed
    # the record of completed objects is kept in id_log.csv
    def get_start_id(self):
        self.data_generation_log = np.loadtxt('./id_log.csv', delimiter=',')
        # the start id is decided based on first zero that is detected in id_log.csv, completed objects are changed to 1
        dataless_object_id = self.find_first_zero()
        
        for object_id in range(self.starting_object_id,len(self.data_generation_log),self.no_of_objects):
            if object_id <= dataless_object_id < object_id + self.no_of_objects:
                self.starting_object_id= object_id + self.no_of_objects
                return object_id
            
    # run the robot_function.py and pass the start_if and no_of_objects as arguments        
    def run_simulation1(self,start_id, no_of_objects):
            # Specify the relative path to slip_data_generator.py
        print('start_id= ', start_id, 'no_of_objects= ', no_of_objects)
        command = f"xvfb-run -a python slip_data_generator.py {start_id} {no_of_objects}"
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        result = stdout.decode('utf-8').strip()  # Capture the result from stdout
        return result == "True"  # Convert the string result to a boolean value

    def manage_treads(self):
        # Create and start threads for each argument
        threads = []
        results = {}
        try:
            while True: 
                running_threads = sum(1 for thread in threads if thread.is_alive())
                # If the number of running threads is lower than expected, start new threads
                while running_threads < self.no_of_threads:
                    #start id will decide from where to start data genration for that particular thread
                    #the thread will generate data for defined number of objects from the start id
                    start_id = self.get_start_id()
                    t = MyThread(target=self.run_simulation1, args=(start_id, self.no_of_objects))
                    threads.append(t)
                    t.start()
                    running_threads += 1

                # Print results for completed threads
                for thread in threads:
                    if not thread.is_alive() and thread.result is not None:
                        print(f"Result for arguments {thread._args[0]}, {self.no_of_objects}: {thread.result}")
                        threads.remove(thread)

        except KeyboardInterrupt:
            print('keyboard interrupt')

    # Ensure that any remaining results are printed
        for thread in threads:
            if thread.result is not None:
                print(f"Result for arguments {thread._args[0]}, {self.no_of_objects}: {thread.result}")

        print("All simulations completed")



run = run_simulation()
# each thread will perform data generation on no_of_objects defined below and then it will terminate
run.no_of_objects = 5
#no of threads that will run in parallel
run.no_of_threads = 5

if __name__ == "__main__":
    run.manage_treads()