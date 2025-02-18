import psutil as ps
import tqdm as tq 
from time import sleep

def percentCPU():
    with tq.tqdm(total=100, desc="cpu%", position=1) as cpubar, tq.tqdm(total=100, desc="ram%", position=0) as rambar:
        while True:
            rambar.n=ps.virtual_memory().percent
            cpubar.n=ps.cpu_percent()
            rambar.refresh()
            cpubar.refresh()

percentCPU()


# print(ps.pids())
# print(p)
# ps.cpu_times_percent()
# print(tqdm.__version__)