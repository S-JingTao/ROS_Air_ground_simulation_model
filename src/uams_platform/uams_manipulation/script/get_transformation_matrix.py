#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2021/1/18:上午10:23
# @Author  : jingtao sun

import rospy
import tf
from tf.transformations import quaternion_matrix
import numpy as np
import geometry_msgs.msg

# 监听TF变换，获得广义坐标系之间的平移和四元数表示的旋转变换，并广播

class GetTransformationMatrix(object):
    def __init__(self):
        self.all_frame_list = ['uams/base_link', 'link1', 'link2',
                               'link3', 'link4', 'end_effector_link']
        # 将各个关节坐标进行编号映射
        self.frame_dict = {
            0: 'uams/base_link',
            1: 'link1',
            2: 'link2',
            3: 'link3',
            4: 'link4',
            5: 'end_effector_link'}

        sorted(self.frame_dict.keys())
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(1.0)
        self.broad = tf.TransformBroadcaster()
        self.temp = geometry_msgs.msg.TransformStamped()

    def is_interlinked_frame(self, link_a, link_b):
        # 判断两个frame是否相通
        if self.listener.canTransform(link_a, link_b, rospy.Time(0)):
            return True
        else:
            return False

    def compute_jacobian_mat(self,mat_list):
        mat_list = [np.transpose(x) for x in mat_list]

        def _stitch_by_row(mat,r_index):
            zero = np.array([[0], [0], [0], [0], [0], [0]])
            I = np.eye(6)
            z = np.array([[0], [0], [0], [0], [0], [1]])

            def _dot(a,b):
                c = np.dot(a,b)
                return c

            if r_index == 1:
                row_temp = I
                for i in range(4):
                    row_temp = np.hstack((row_temp,zero))
            elif r_index == 2:
                row_temp = np.hstack((mat[0], z))
                for i in range(3):
                    row_temp = np.hstack((row_temp, zero))
            elif r_index == 3:
                row_temp = np.hstack((mat[1], _dot(mat[5],z)))
                row_temp = np.hstack((row_temp,z))
                for i in range(2):
                    row_temp = np.hstack((row_temp, zero))
            elif r_index == 4:
                row_temp = np.hstack((mat[2], _dot(mat[6], z)))
                row_temp = np.hstack((row_temp, _dot(mat[9], z)))
                row_temp = np.hstack((row_temp, z))
                row_temp = np.hstack((row_temp, zero))
            elif r_index ==5:
                row_temp = np.hstack((mat[3], _dot(mat[7], z)))
                row_temp = np.hstack((row_temp, _dot(mat[10], z)))
                row_temp = np.hstack((row_temp, _dot(mat[12], z)))
                row_temp = np.hstack((row_temp, z))
            elif r_index == 6:
                row_temp = np.hstack((mat[4], _dot(mat[8], z)))
                row_temp = np.hstack((row_temp, _dot(mat[11], z)))
                row_temp = np.hstack((row_temp, _dot(mat[13], z)))
                row_temp = np.hstack((row_temp, _dot(mat[14], z)))
            else:
                raise ValueError('R_INDEX ERROR!')
            return row_temp
        J_b = _stitch_by_row(mat_list, 1)
        for i in range(2,7):
            J_b =np.vstack((J_b,_stitch_by_row(mat_list, i)))

        return J_b

    def broad_trans(self, trans, quaternion, link_a, link_b):
        self.temp.header.frame_id = link_a
        self.temp.header.stamp = rospy.Time(0)
        self.temp.child_frame_id = link_b

        self.temp.transform.translation.x = trans[0]
        self.temp.transform.translation.y = trans[1]
        self.temp.transform.translation.z = trans[2]
        self.temp.transform.rotation.w = quaternion[0]
        self.temp.transform.rotation.x = quaternion[1]
        self.temp.transform.rotation.y = quaternion[2]
        self.temp.transform.rotation.z = quaternion[3]

        self.broad.sendTransformMessage(self.temp)
        print(link_a)
        print(link_b)
        print(trans)
        print(quaternion)

        print('OK')
        print('------')
        self.rate.sleep()

    def get_trans_rot(self, link_father, link_child):
        for i in range(len(link_father)):
            self.listener.waitForTransform(link_father[i], link_child[i], rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            trans_mat_list = []
            try:
                for i in range(len(link_father)):
                    # 返回的是平移向量和四元数表示的旋转矩阵
                    (trans, quaternion) = self.listener.lookupTransform(link_father[i], link_child[i], rospy.Time(0))
                    print(i)
                    print(link_father[i])
                    print(link_child[i])
                    print(self.get_generalize_trans_mat(trans, quaternion))
                    # rospy.sleep(5)

                    trans_mat_list.append(self.get_generalize_trans_mat(trans, quaternion))
                    rospy.loginfo('距离原点的位置: x=%f ,y= %f，z=%f \n 旋转四元数: w=%f ,x= %f，y=%f z=%f ', trans[0], trans[1],
                                  trans[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            print('矩阵数量：', len(trans_mat_list))
            J_b = self.compute_jacobian_mat(trans_mat_list)
            print('-------------')
            print(J_b)
            rospy.sleep(5)

        # self.listener.waitForTransform(link_father, link_child, rospy.Time(), rospy.Duration(4.0))
        #
        # try:
        #     # 返回的是平移向量和四元数表示的旋转矩阵
        #     (trans, quaternion) = self.listener.lookupTransform(link_father, link_child, rospy.Time(0))
        #     print(type(trans))
        #     print(type(quaternion))
        #     # self.broad_trans(trans,quaternion,link_a, link_b)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass

    def get_generalize_trans_mat(self, trans, quaternion):
        # 得到广义变换矩阵
        zero = np.array([[0, 0, 0],
                         [0, 0, 0],
                         [0, 0, 0]])
        rot = np.array(quaternion_matrix(quaternion)[0:3,0:3])
        s_r = np.array([[0, -trans[0], trans[1]],
                        [trans[0], 0, -trans[0]],
                        [-trans[1], trans[0], 0]])

        s_rot = np.dot(s_r, rot)
        temp_1= np.concatenate([rot, zero],1)
        temp_2 = np.concatenate([s_rot, rot],1)
        trans_mat = np.vstack((temp_1,temp_2))
        return trans_mat

    def get_converter(self):
        father_frame = []
        child_frame = []
        for i in range(len(self.all_frame_list) - 1):
            print(i)
            for j in range(i + 1, len(self.all_frame_list)):
                # print(self.frame_dict[i])
                # print(self.frame_dict[j])
                # print('---------------')
                father_frame.append(self.frame_dict[i])
                child_frame.append(self.frame_dict[j])

                # while self.is_interlinked_frame(self.frame_dict[i],self.frame_dict[j]):
        self.get_trans_rot(father_frame, child_frame)


def main():
    rospy.init_node('get_transformation_node', anonymous=False)
    get_trans = GetTransformationMatrix()
    try:
        get_trans.get_converter()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
