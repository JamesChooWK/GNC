#!/usr/bin/env python

import rospy
from gdp.msg import Order
from gdp.msg import OrderList
import time

order1 = Order()
order2 = Order()
order3 = Order()
order4 = Order()
orderList = OrderList()

def mission1():

    print("Mission 1")

    order1.idAgent = 1 
    order1.order_name = 1
    order1.order_info = [1,1,1] 
    order1.timestamp = time.time()

    order2.idAgent = 1
    order2.order_name = 2
    order2.order_info = [47.3977417, 8.5455928, 2.7] 
    order2.timestamp = time.time()

    order3.idAgent = 1
    order3.order_name = 5
    order3.order_info = [1,1,1]
    order3.timestamp = time.time()

    orderList.idAgent = 1
    orderList.order = [order1, order2, order3]

def mission2():

    print("Mission 2")

    order1.idAgent = 1 
    order1.order_name = 1
    order1.order_info = [1,1,1] 
    order1.timestamp = time.time()

    order2.idAgent = 1
    order2.order_name = 2
    order2.order_info = [47.3977417, 8.5455928, 5.0] 
    order2.timestamp = time.time()
   
    #order1.idAgent = 1
    #order1.order_name = 2
    #order1.order_info = [47.3977417+0.00005, 8.5455928+0.00005, 5.0]
    #order1.timestamp = time.time()

    #order2.idAgent = 1
    #order2.order_name = 2
    #order2.order_info = [47.3977417-0.00005,8.5455928-0.00005,5.0]
    #order2.timestamp = time.time()	
	
    order3.idAgent = 1
    order3.order_name = 2
    order3.order_info = [47.3977417+0.00005, 8.5455928+0.00005, 5.0]
    order3.timestamp = time.time()

    order4.idAgent = 1
    order4.order_name = 2
    order4.order_info = [47.3977417-0.00005,8.5455928-0.00005,5.0]
    order4.timestamp = time.time()
    
    

    orderList.idAgent = 1
    orderList.order = [order1, order2, order3, order4]

def mission3():

    print("Mission 3")

    order1.idAgent = 1
    order1.order_name = input("Order1: ")
    order1.order_info = input("OrderInfo1 :")
    order1.timestamp = time.time()

    order2.idAgent = 1
    order2.order_name = input("Order2: ")
    order2.order_info = input("OrderInfo2 :")
    order2.timestamp = time.time()

    order3.idAgent = 1
    order3.order_name = input("Order3: ")
    order3.order_info = input("OrderInfo3 :")
    order3.timestamp = time.time()

    orderList.idAgent = 1
    orderList.order = [order1, order2, order3]

def talker():
    pub = rospy.Publisher('Orders', OrderList, queue_size=10)
    rospy.init_node('TaskAllocation', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():

        entry = int(input("Which Mission? : "))

        if entry == 1:
            mission1()
        elif entry == 2:
            mission2()
        else:
            mission3()


        pub.publish(orderList)

        print("Message Sent")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
