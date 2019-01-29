import sys
import matplotlib.pyplot as plt
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
#print(x)
#print(y) 


##打印地图中的数据
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
plt.plot(x1, y1, color="b",  label='map')
plt.plot(x, y, color="r",  marker="*",  label='lanep_point')
plt.legend()
plt.show()
