#!/usr/bin/env python

import matplotlib.pyplot as plt
import math

def inverse_kinematics_data():
    joint_data=dict()
    for i in range(7):
        joint_data['joint'+str(i)]=list()

    with open('desired_joint_values.txt','r') as f:
        for line in f.readlines():
            line_data=list(map(float,line.split()))
            for i in range(7):
                joint_data['joint'+str(i)].append(line_data[i])


    index=[i for i in range(len(joint_data['joint0']))]

    plt.figure(1)
    for i in range(7):
        plt.plot(index,joint_data['joint'+str(i)],label='joint'+str(i))
    
    plt.legend(loc='upper right')
    plt.show()

def interpolation_data():
    joint_data=dict()
    acc=list()
    for i in range(7):
        joint_data['joint'+str(i)]=dict()
        joint_data['joint'+str(i)]['pos']=list()
        joint_data['joint'+str(i)]['vel']=list()
        
    with open('interpolated_values.txt','r') as f:
        for line in f.readlines():
            line_data=list(map(float,line.split()))
            for i in range(len(line_data)/2):
                joint_data['joint'+str(i)]['pos'].append(line_data[2*i])
                joint_data['joint'+str(i)]['vel'].append(line_data[2*i+1])
            acc.append(line_data[-1])


    index=[i for i in range(len(joint_data['joint0']['pos']))]

    plt.figure(1)
    for i in range(7):
        plt.plot(index,joint_data['joint'+str(i)]['pos'],label='joint'+str(i)+'pos')
        plt.plot(index,joint_data['joint'+str(i)]['vel'],label='joint'+str(i)+'vel')
    #plt.plot(index,acc,label='acc')
    plt.legend(loc='upper right')
    plt.show()

def state_data():
    joint_data=dict()
    for i in range(7):
        joint_data['joint'+str(i)]=dict()
        joint_data['joint'+str(i)]['pos']=list()
        joint_data['joint'+str(i)]['vel']=list()
        
    with open('states.txt','r') as f:
        for line in f.readlines():
            line_data=list(map(float,line.split()))
            for i in range(len(line_data)/2):
                joint_data['joint'+str(i)]['pos'].append(line_data[2*i])
                joint_data['joint'+str(i)]['vel'].append(line_data[2*i+1])


    index=[i*20.0/len(joint_data['joint0']['pos']) for i in range(len(joint_data['joint0']['pos']))]

    plt.figure(1)
    for i in range(7):
        plt.plot(index,joint_data['joint'+str(i)]['pos'],label='joint'+str(i)+'pos')
        #plt.plot(index,joint_data['joint'+str(i)]['vel'],label='joint'+str(i)+'vel')
    #plt.plot(index,acc,label='acc')
    plt.legend(loc='upper right')
    plt.show()

def plot_diff():
    joint_data=dict()
    for i in range(7):
        joint_data['joint'+str(i)]=dict()
        joint_data['joint'+str(i)]['pos']=list()
        joint_data['joint'+str(i)]['vel']=list()
        
    with open('states.txt','r') as f:
        for line in f.readlines()[50:]:
            line_data=list(map(float,line.split()))
            for i in range(len(line_data)/2):
                joint_data['joint'+str(i)]['pos'].append(line_data[2*i])
                joint_data['joint'+str(i)]['vel'].append(line_data[2*i+1])

    index=[i*20.0/len(joint_data['joint0']['pos']) for i in range(len(joint_data['joint0']['pos']))]


    desired_data=dict()
    for i in range(7):
        desired_data['joint'+str(i)]=list()

    with open('desired_joint_values.txt','r') as f:
        for line in f.readlines():
            line_data=list(map(float,line.split()))
            for i in range(7):
                desired_data['joint'+str(i)].append(line_data[i])


    index_d=[i*0.1 for i in range(len(desired_data['joint0']))]

    plt.figure(1)
    for i in range(7):
        plt.plot(index,joint_data['joint'+str(i)]['pos'],label='joint'+str(i)+'pos')
        plt.plot(index_d,desired_data['joint'+str(i)],label='joint'+str(i))
        #plt.plot(index,joint_data['joint'+str(i)]['vel'],label='joint'+str(i)+'vel')
    #plt.plot(index,acc,label='acc')
    plt.legend(loc='upper right')
    plt.show()


if __name__=='__main__':
    #inverse_kinematics_data()
    #interpolation_data()
    #state_data()
    plot_diff()