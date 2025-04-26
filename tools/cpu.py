import psutil as ps # type: ignore
import tqdm as tq  # type: ignore
from statistics import *
from time import sleep

def percentCPU():
    with tq.tqdm(total=100, desc="cpu%", position=1) as cpubar, tq.tqdm(total=100, desc="ram%", position=0) as rambar:
        while True:
            rambar.n=ps.virtual_memory().percent
            cpubar.n=mean(ps.cpu_percent(interval=0.1, percpu=True))
            rambar.refresh()
            cpubar.refresh()
            sleep(0.1)

percentCPU()


# print(ps.pids())
# print(p)
# ps.cpu_times_percent()
# print(tqdm.__version__)