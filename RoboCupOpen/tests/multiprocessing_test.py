from multiprocessing import Process, Manager, Value
import time
import ctypes

# class Variables:
#     def __init__(self):
#         self.a = Value('i')
#         self.b = Value('i')

class Variables:
    a = Value('i')
    b = Value('i')

def f(id, data):
    print(id, data.a)
    while True:
        print(id, data.a.value)
        data.a.value += 1
        time.sleep(id)

if __name__ == '__main__':
    with Manager() as manager:
        data = Variables
        print(data.a)

        p1 = Process(target=f, args=(1, data))
        p2 = Process(target=f, args=(2, data))

        start = time.time()
        p1.start()
        p2.start()
        p1.join()
        p2.join()

        print(time.time()-start)
