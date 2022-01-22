function [v,a]=velocity(path,Ts)
vmax=0.5;
k_max = 0.2;
k_min = 0.01; 
factor= 0.5;
rx = path(:,1);
ry = path(:,2);
ryaw = path(:,3);
v=[];
a=[];
flag=[];
for i = 1:length(rx)
   if i == 1
       v(i) = (rx(i+1) - rx(i))/(Ts)*cos(ryaw(i))+(ry(i+1)-ry(i))/(Ts)*sin(ryaw(i));
       flag(i)=1;
       continue
   elseif i == length(rx)
       v(i) = 0;
       break
   end
   
   pt1.x = rx(i-1);
   pt1.y = ry(i-1);
   pt2.x = rx(i);
   pt2.y = ry(i);
   pt3.x = rx(i+1);
   pt3.y = ry(i+1);
   
   k = CalcCur(pt1,pt2,pt3);
   % 转弯半径大时，可用最高速度行驶
   if k <= k_min
      v(i)=vmax;
   % 转弯半径小时，用最低速度
   elseif k>=k_max
      v(i)=1*factor;
   % 否则介于两者之间
   else
      k=k-k_min;
      v(i)=(vmax-1)*k+1;
      v(i)=v(i)*factor;
   end
   
   
   theta1 = atan2(pt2.y-pt1.y,pt2.x-pt1.x);
   theta2 = atan2(pt3.y-pt2.y,pt3.x-pt2.x);
   if abs(theta2 - theta1)>3*pi/4
       flag(i)=-flag(i-1);
   else
       flag(i)=flag(i-1);
   end
   v(i) = v(i)*flag(i);
end
   v = v';
   a = diff(v);
   a = a/Ts;
end
function k=CalcCur(pt1,pt2,pt3)
k_1=(pt2.y-pt1.y)/(pt2.x-pt1.y);   
k_2=(pt2.y-pt1.y)/(pt2.x-pt1.y);
if k_1 == 0
    k_1 = 0.0001;
end
if k_2 == 0
    k_2 = 0.0002;
end
center.x=(k_1*k_2*(pt1.y-pt3.y)+k_1*(pt1.x+pt2.x)-k_2*(pt2.x+pt3.x))/(2*(k_2-k_1));
center.y=-(center.x-(pt1.x+pt2.x)/2)*k_1 +(pt1.y+pt2.y)/2;
center1=[center.x center.y];
pt = [pt1.x pt1.y];
k=1/norm(center1-pt);
end