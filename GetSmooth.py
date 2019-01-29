import sys
import matplotlib.pyplot as plt
import re

HDx=[]
HDy=[]
Sx=[]
Sy=[]
fb = open('./finalpoint.txt','r')
lines=fb.readlines()
#print(lines)
try: 
    for line in lines:
        #print(line)
        t1=re.findall(r'(\(0\):\d+)',line)#匹配S(0):587510 
        #print(t1)
        if len(t1)>0:
            line1=int(t1[0].split(':')[-1])
            Sx.append(line1)
            #print(line1)
            #print(type(line1))
        t2=re.findall(r'(\(1\):\d*\.\d+)',line)#匹配S(1):587510 
        if len(t2)>0:
            line2=int(float(t2[0].split(':')[-1])*1000000)
            Sy.append(line2)
            #print(line2)
           # print(type(line2))
        t3=re.findall(r'(x\(\):\d+)',line)#匹配hdmap_point.x():587508 
        if len(t3)>0:
            line3=int(t3[0].split(':')[-1])
            HDx.append(line3)
        #print(line3)
        #print(type(line3))
        t4=re.findall(r'(y\(\):\d+\.\d+)',line)#匹配hdmap_point.y():4.14064e+06
        if len(t4)>0:
            line4=int(float(t4[0].split(':')[-1])*1000000)
            HDy.append(line4)
            #print(line4)
            #print(type(line4))
finally:
     fb.close()
#print(Sx)
#print(Sy) 
#print(len(Sx))
#print(len(Sy))
x=[]
y=[]
fb = open('./a.txt','r')
lines=fb.readlines()
try: 
    for line in lines:
        ｌine1=line.strip()
        if line1.startswith('x') :# 变量初始化
            line2=line1.split()[1]
            x.append(line2)
                        
        if line1.startswith('y'):
            line3=line1.split()[1]
            y.append(line3)                      
finally:
     fb.close()

x1=[]
y1=[]
fb1 = open('./map.txt','r')
lines1=fb1.readlines()
try: 
    for line in lines1:
        ｌine1=line.strip()
        if line1.startswith('x') :# 变量初始化
            line2=line1.split()[1]
            #print(line2)
            x1.append(line2)
                        
        if line1.startswith('y'):
            line3=line1.split()[1]
            #print(line3)
            y1.append(line3)                      
finally:
     fb.close()
ｘ = list(map(eval, ｘ))
y = list(map(eval, y))        
ｘ1 = list(map(eval, ｘ1))
y1 = list(map(eval, y1))
plt.plot(x1, y1, color="b", marker=">", label='Oringin_map')
plt.plot(x, y, color="y",  marker="D",  label='lanep_point')
#plt.legend()
#plt.show()
plt.plot(HDx, HDy, color="r",  marker="o", linewidth=1.0, label='HD_map')
plt.plot(Sx, Sy, color="g",  marker="*", linewidth=1.0, label='Smooth')
plt.legend()
plt.show()