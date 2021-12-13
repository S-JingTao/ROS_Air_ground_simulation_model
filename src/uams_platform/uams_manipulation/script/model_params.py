# -*- coding: utf-8 -*-
# @Time    : 2021/1/18:下午2:57
# @Author  : jingtao sun
import rospy

class GetParams(object):
    def __init__(self):
        pass
    def get_converter(self):



def main():
    rospy.init_node('get_transformation_node', anonymous=False)
    get = GetParams()
    try:
        get.get_converter()
    except rospy.ROSInterruptException:
        pass
    pass

if __name__ == '__main__':
    main()