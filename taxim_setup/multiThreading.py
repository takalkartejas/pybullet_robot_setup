import threading
import queue
import worker


def runSimulation(numberOfCPUs,operating_system,command,number_cycles,first_cycle,shapeDropLimit,splitCycles=False,nb_cycles_skip=0):        
        #if first_cycle==number_cycles: first_cycle=number_cycles-1
        splitCycles=False # to be removed after testing
        affinity = 1
        isSecondary = ""
        threads = []
        cyclesPerCPU = math.ceil(number_cycles/ numberOfCPUs)
        fullCPUs =  numberOfCPUs -((cyclesPerCPU *  numberOfCPUs) - number_cycles)
        #for example numberOfCPUs = 12, number_cycles = 500, cyclesPerCPU_wo_ceil = 41.67, cyclesPerCPU = 42
        # 12 - ((504) - 500) = 12- 4 = 8 full cpus
        lastC = first_cycle+cyclesPerCPU-1
        firstC=first_cycle       
        # Evenly spread continuous chunks of cycles on each simulation
        if splitCycles and numberOfCPUs>number_cycles:
            cyclesPerSubCPU = math.ceil((number_cycles*shapeDropLimit)/ numberOfCPUs)
            cyclesPerCPU = math.ceil(number_cycles/ numberOfCPUs)
            fullSubCPUs =  numberOfCPUs -((cyclesPerSubCPU *  numberOfCPUs) - number_cycles*shapeDropLimit)
            #what is shareFropLimit?
            lastSubC = 0+cyclesPerSubCPU-1
            firstSubC=0
            for i in range( numberOfCPUs):
                if firstC>number_cycles:
                    break
                if firstSubC>number_cycles*shapeDropLimit:
                    break
                if i >= fullSubCPUs:
                    if cyclesPerSubCPU <= 1:
                        break
                    lastSubC = lastSubC - 1
                
                if operating_system=="Windows":
                    affinityStr = hex(affinity)
                else:
                    affinityStr = str(affinity)
                str_headless='-h'*(i!=1) # Run the first simulation in non-headless mode to be able to see what is going on
                #str_headless=' '
                print(F"  Spawn simulation simulating {i} from cycles {firstC+nb_cycles_skip} to {lastC} with  {firstSubC} to {lastSubC} with CPU affinity: {affinityStr}", flush=True)
                
                count_h = command.count("-h")
                # If there are more than one occurrence of "-h", remove one occurrence
                if count_h > 1:
                    command=command.replace("-h", "", count_h-1)
                if operating_system=="Windows":
                    t=threading.Thread(target=os.system, args=('start /WAIT /AFFINITY ' + affinityStr + ' ' + command.format(str_headless,firstC+self.nb_cycles_skip,lastC,firstSubC,lastSubC,isSecondary),), daemon=True)
                else:
                    #t=threading.Thread(target=os.system, args=('taskset -c ' + affinityStr + ' ' + command.format(str_headless,firstC+self.nb_cycles_skip,lastC,firstSubC,lastSubC,isSecondary),), daemon=True)
                    t = threading.Thread(target=os.system, args=(f'chrt -a --pid {affinityStr} ; {command.format(str_headless,firstC+self.nb_cycles_skip,lastC,firstSubC,lastSubC,isSecondary)}',), daemon=True)
                   
                affinity = affinity * 2 #4 #2
                isSecondary = ' -GisSecondary=true'
                threads.append(t)
                t.start()
                firstSubC = lastSubC + 1
                lastSubC = min(lastSubC +1+cyclesPerSubCPU,shapeDropLimit)
                if firstSubC>shapeDropLimit:
                    lastSubC = first_cycle+cyclesPerSubCPU-1
                    firstSubC=0
                    firstC = lastC + 1
                    lastC = min(lastC + cyclesPerCPU,number_cycles)
        else:
            for i in range( numberOfCPUs):
                if firstC>number_cycles:
                    break
                if i >= fullCPUs:
                    if cyclesPerCPU <= 1:
                        break
                    lastC = lastC - 1
                if operating_system=="Windows":
                    affinityStr = hex(affinity)
                else:
                    affinityStr = str(affinity)
                str_headless='-h'*(i!=0) # Run the first simulation in non-headless mode to be able to see what is going on
           
                print(F"  Spawn simulation simulating from cycles {firstC+nb_cycles_skip} to {lastC} with CPU affinity: {affinityStr}", flush=True)
                count_h = command.count("-h")
                # If there are more than one occurrence of "-h", remove one occurrence
                
                if count_h > 1:
                    command=command.replace("-h", "", count_h-1)
                if operating_system=="Windows":
                    t=threading.Thread(target=os.system, args=('start /WAIT /AFFINITY ' + affinityStr + ' ' + command.format(str_headless,firstC+nb_cycles_skip,lastC,' ',' ',isSecondary),), daemon=True)
                else:
                    #t=threading.Thread(target=os.system, args=('taskset -c ' + affinityStr + ' ' + command.format(str_headless,firstC+nb_cycles_skip,lastC,' ',' ',isSecondary),), daemon=True)
                    t = threading.Thread(target=os.system, args=(f'chrt -a --pid {affinityStr} ; {command.format(str_headless, firstC + nb_cycles_skip, lastC, " ", " ", isSecondary)}',), daemon=True)
                   
                threads.append(t)
                t.start()


                # Multiplication with 4 is equal to a left shift by two bits
                affinity = affinity * 2 #4 #2
                isSecondary = ' -GisSecondary=true'
                firstC = lastC + 1
                lastC = min(lastC + cyclesPerCPU,number_cycles)
                
        for t in threads:
            t.join()

def run_calculation(number):
    result = worker.calculate_square(number)
    print(f"Square of {number}: {result}")

# Define the arguments for each thread
arguments = [2, 3, 4]

# Create and start threads for each argument
threads = []
for arg in arguments:
    t = threading.Thread(target=run_calculation, args=(arg,))
    threads.append(t)
    t.start()

# Wait for all threads to finish
for t in threads:
    t.join()

print("All calculations completed")