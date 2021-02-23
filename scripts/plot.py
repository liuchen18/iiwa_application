#!/usr/bin/env python

import matplotlib.pyplot as plt
import math

def plot_vel():
    vel=list()
    vel_d=list()
    pos=list()
    with open('vel.txt','r') as f:
        for line in f.readlines()[1:]:
            vel.append(float(line))

    with open('pos.txt','r') as f:
        for line in f.readlines()[1:]:
            pos.append(float(line))
    with open('vel_d.txt','r') as f:
        for line in f.readlines()[1:]:
            vel_d.append(float(line))
    index=[i*10.0/len(vel) for i in range(len(vel))]

    index_d=[i*1.0/10000 for i in range(len(vel_d))]

    index_p=[i*10.0/len(pos) for i in range(len(pos))]

    plt.figure(1)
    plt.plot(index,vel,label='vel_a')
    plt.plot(index_d,vel_d,label='vel_d')
    plt.plot(index_p,pos,label='pos')
    plt.ylim(-0.5,0.5)
    plt.show()

def plot_sigmoid():
    sh=0.1
    sl=0.0
    S=[]
    x=[]
    for i in range(1000):
        x.append(i)
        sx=sl+(sh-sl)/(1.0+math.pow(math.e,-5*(2*(i*1.0+1)-1000)/1000))
        S.append(sx)
    plt.plot(x,S)
    plt.show()

def plot_pos():
    pos=list()
    vel=list()
    with open('pos.txt','r') as f:
        for line in f.readlines()[1:]:
            pos.append(float(line))
    with open('vel.txt','r') as f:
        for line in f.readlines()[1:]:
            vel.append(float(line))

    index_p=[i*10.0/len(pos) for i in range(len(pos))]
    index_v=[i*10.0/len(vel) for i in range(len(vel))]

    plt.figure(1)
    plt.plot(index_p,pos,label='position')
    plt.plot(index_v,vel,label='velocity')
    plt.legend(loc='upper right')
    plt.show()

def plot_posvel():
    vel=[list(),list(),list()]
    pos=[list(),list(),list()]
    vel_d=list()
    pos_d=list()
    with open('posvel.txt','r') as f:
        for line in f.readlines()[1:]:
            vel[0].append(float(line.split()[1]))
            vel[1].append(float(line.split()[3]))
            vel[2].append(float(line.split()[5]))
            pos[0].append(float(line.split()[0]))
            pos[1].append(float(line.split()[2]))
            pos[2].append(float(line.split()[4]))
    with open('posvel_d.txt','r') as f:
        for line in f.readlines()[1:]:
            vel_d.append(float(line.split()[1]))
            pos_d.append(float(line.split()[0]))

    index=[i*10.0/len(vel[0]) for i in range(len(vel[0]))]
    index_d=[i*10.0/len(vel_d) for i in range(len(vel_d))]

    plt.figure(1)
    plt.plot(index,vel[0],label='vel_5')
    plt.plot(index,vel[1],label='vel_4')
    plt.plot(index,vel[2],label='vel_2')
    plt.plot(index,pos[0],label='pos_5')
    plt.plot(index,pos[1],label='pos_4')
    plt.plot(index,pos[2],label='pos_2')
    plt.plot(index_d,vel_d,label='vel_d')
    plt.plot(index_d,pos_d,label='pos_d')
    plt.legend(loc='lower left')
    plt.ylim(-0.1,0.3)
    plt.show()

def plot_moveit_command():
    pos=list()
    time=list()
    pos_s=list()
    time_s=list()
    init_time=0
    with open('pos_command.txt','r') as f:
        for line in f.readlines()[1:]:
            pos.append(float(line.split()[1]))
            if len(time) == 0:
                time.append(0)
                init_time=float(line.split()[0])
            else:
                time.append(float(line.split()[0])-init_time)

    with open('pos_state.txt','r') as f:
        for line in f.readlines()[1:]:
            pos_s.append(float(line.split()[1]))
            if len(time_s) == 0:
                time_s.append(0)
                init_time=float(line.split()[0])
            else:
                time_s.append(float(line.split()[0])-init_time)

    plt.figure(1)
    plt.plot(time,pos,label='command')
    plt.plot(time_s,pos_s,label='state')
    plt.legend(loc='lower left')
    #plt.ylim(-0.1,0.2)
    plt.show()

if __name__ == '__main__':
    plot_vel()
    #plot_sigmoid()
    #plot_pos()
    #plot_posvel()
    #plot_moveit_command()