import asyncio
import paho.mqtt.client as paho
import time
import random


"""PERCOBAAN !"""
# T1 = 1
# T2 = 2
# T3 = 3
# t1 = 0
# t2 = 0
# t3 = 0
# T4 = 100
# t4= {}
# data = ()

# async def data_database():
#     global t1,t2,t3, data_database
#     t1 = T1
#     t2 = t2
#     t3 = T3
#     t4 = T4
#     return t1,t2,t3,t4

# async def dic_database():
#     print(type(data_database()))
#     await asyncio.sleep(1)

# async def push_database():
#     t4 = T2

# async def main():
#     while True:
#         taskA = loop.create_task(data_database())
#         taskB = loop.create_task(dic_database())
#         taskC = loop.create_task(push_database())
#         await asyncio.wait([taskA,taskB,taskC])

# loop = asyncio.get_event_loop()
# loop.run_until_complete(main())
"""PERCOBAANN"""
# T1 = 1
# T2 = 2
# T3 = 3
# t1 = 0
# t2 = 0
# t3 = 0
# T4 = 100
# def data():
#     global t1,t2,t3, data_database
#     t1 = T1
#     t2 = t2
#     t3 = T3
#     t4 = T4
#     return t1,t2,t3,t4

# print(type(data()))

"""yang mif"""
import asyncio
import time

a = 10
b = 20
c = 30
# data_list = []
time_now = 0
time_prev = time.time()
waktu_baru = 0
waktu_lama = time.time()
time_prevs = time.time()


async def data():
    # global ketinggian
    ketinggian = a
    jarak = b
    latensi = c
    # print(ketinggian)
    time_now = time.time() - time_prev
    # time.sleep(1)
    
    print(f"data: {time_now}")
    return ketinggian, jarak, latensi

# print(type(data()))


async def baru():
    # global ketinggian
    coro = data()
    value = await coro
    ketinggian = {"ketinggian": value[0]}
    jarak = {"jarak": value[1]}
    # print(ketinggian)
    # await asyncio.sleep(3)
    # time.sleep(2)
    waktu_sekarang = time.time() - waktu_lama
    print(f"baru:{waktu_sekarang}")
    return ketinggian, jarak


async def push():
    # global ketinggian
    coro = baru()
    value = await coro
    jarak = value[1]
    # print(jarak)
    time_nows = time.time() - time_prevs
    print(f"push :{time_nows}")
    return jarak


async def main():
    # while True:
    task1 = loop.create_task(data())
    task2 = loop.create_task(baru())
    task3 = loop.create_task(push())
    await asyncio.wait([task1, task3, task2])

loop = asyncio.get_event_loop()
loop.run_until_complete(main())
"""INIII BENERRRRR"""
# a = 10
# b = 20
# c = 30

# time_now = 0
# time_prev = time.time()

# waktu_baru = 0
# waktu_lama = time.time()

# async def data():
#     # global ketinggian
#     ketinggian = a
#     jarak = b
#     latensi = c
#     time_now = time.time() - time_prev
#     print(time_now)
#     return ketinggian, jarak, latensi

# async def baru():
#     # global ketinggian
#     coro = data()
#     value = await coro
#     ketinggian = {"ketinggian": value[0]}
#     jarak = {"jarak":value[1]}
#     print(ketinggian)
#     print(jarak)
#     waktu_sekarang = time.time() - waktu_lama
#     print(waktu_sekarang)
#     # await asyncio.sleep(1)

# async def main():
#     # while True:
#     task1 = loop.create_task(data())
#     task2 = loop.create_task(baru())
#     await asyncio.wait([task2,task1])


# loop = asyncio.get_event_loop()
# loop.run_until_complete(main())
# loop.close()
"""PERCOBAAN 2"""
# a = 10
# b = 20
# c = 30

# async def data():
#     global data_dic
#     data_dic = {"ketinggian": 0, "jarak": 0, "latensi": 0}
#     data_dic["ketinggian"] = a
#     data_dic["jarak"] = b
#     data_dic["latensi"] = c

#     # ketinggian = {"ketinggian": a}
#     # jarak = {"jarak":b}
#     # latensi = {"latensi":c}
#     # data_list = [ketinggian,jarak,latensi]
#     return data_dic

# async def baru():
#     print(data_dic["ketinggian"])
    
#     print("task 2")

# async def main():
#     # while True:
#     task1 = loop.create_task(data())
#     task2 = loop.create_task(baru())
#     await asyncio.wait([task1,task2])

# loop = asyncio.get_event_loop()
# loop.run_until_complete(main())






# if __name__ == "__main__":
#     try:
#         loop = asyncio.get_event_loop()
#         loop.run_until_complete(main())
#     except :
#         pass