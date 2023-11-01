clc;
clear all;
lcaja=15*1.33;
r=sqrt(2*(lcaja/4)*(lcaja/4))-(lcaja/4);
rotacion=15;
[x,y]=octavocirculo(-lcaja/2,0,r,5);
[x1,y1]=mediocirculo(-lcaja/4,lcaja/4,lcaja/4,25);
[x2,y2]=octavocirculo2(0,lcaja/2,r,5);
%figure(1)
%plot(x,y,"-o")
%hold on
%plot(x1,y1,"-o")
%plot(x2,y2,"-o")
%hold off

matriz=[x x1(2:end) x2(2:end); y y1(2:end) y2(2:end)];
%plot(matriz(1,:),matriz(2,:),"-o")
theta = -90; % to rotate 90 counterclockwise
R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
matriz1=R*matriz;

%plot(matriz1(1,:),matriz1(2,:),"-o")
matriz2=R*matriz1;
matriz3=R*matriz2;
matrizt=[matriz(1,:) matriz1(1,2:end) matriz2(1,2:end) matriz3(1,2:end);
         matriz(2,:) matriz1(2,2:end) matriz2(2,2:end) matriz3(2,2:end)];
plot(matrizt(1,:),matrizt(2,:),"-o")

theta=rotacion;
R2 = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
matrizt=R2*matrizt;
matrizt=matrizt+max(max(matrizt));
plot(matrizt(1,:),matrizt(2,:),"-o");

matrizt=matrizt+7;
figure
plot(matrizt(1,:),matrizt(2,:),"-o")

figure
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
%largo brazos
L1 = 22;
L2 = 18;

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

q0 = homeConfiguration(robot);
ndof = length(q0);
count= length(matrizt(1,:));

qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = [matrizt(1,i) matrizt(2,i) 0];
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = [matrizt(1,i) matrizt(2,i) 0];
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end



%{
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(matrizt(1,:),matrizt(2,:),'k')
axis([-30 40 -30 40])


framesPerSecond = 60;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
%}
%tiempo mas lento 80 s
t=linspace(0,80,length(matrizt(1,:)));
figure
plot(t,qs(:,1)*180/pi)
hold on 
plot(t,qs(:,2)*180/pi)
hold off
%{
w1(1)=0;
w2(1)=0;
for i=1:length(matrizt(1,:))-1
    w1(i+1)=(qs(i+1,1)-qs(i,1))/t(2);
    w2(i+1)=(qs(i+1,2)-qs(i,2))/t(2);
end

figure
plot(t,w1)
hold on 
%plot(t,w2)


a1(1)=0;
a2(1)=0;
for i=1:length(matrizt(1,:))-1
    a1(i+1)=(w1(i+1)-w1(i))/t(2);
    a2(i+1)=(w2(i+1)-w2(i))/t(2);
end

figure
plot(t,a2)
hold on 
%plot(t,a2)
%hold off
%}
%{
j1(1)=0;
j2(1)=0;
for i=1:length(matrizt(1,:))-1
    j1(i+1)=(a1(i+1)-a1(i))/t(2);
    j2(i+1)=(a2(i+1)-a2(i))/t(2);
end
figure
plot(t,j2)
hold on 
%plot(t,j2)
%hold off
%}
m1=qs(:,1);
m2=qs(:,2);
%motor2

a02 =      -1.638 ;
       a12 =     -0.6139;%  (-0.6196, -0.6082)
       b12 =      0.3608;%  (0.351, 0.3706)
       a22 =   -0.004452;%  (-0.008621, -0.0002824)
       b22 =    0.004955;%  (0.002416, 0.007493)
       a32 =     0.06134;%  (0.05725, 0.06544)
       b32 =     0.05276;%  (0.04723, 0.05828)
       a42 =    -0.05557;%  (-0.05811, -0.05303)
       b42 =   0.0009292;%  (-0.0007072, 0.002566)
       a52 =      0.0808;%  (0.07773, 0.08386)
       b52 =    -0.04647;%  (-0.05482, -0.03812)
       a62 =  -0.0002976;%  (-0.002153, 0.001558)
       b62 =    0.001955;%  (0.0004206, 0.003489)
       a72 =    0.007615;%  (0.005818, 0.009411)
       b72 =   -0.002709;%  (-0.004605, -0.0008126)
       w2 =     0.07848 ;% (0.07805, 0.07891)




%motor 1

a0 =       1.485;%  (1.483, 1.487)
       a1 =      0.4383;%  (0.4338, 0.4427)
       b1 =      0.1876;%  (0.1793, 0.1958)
       a2 =     0.05094;%  (0.0495, 0.05238)
       b2 =      0.0547;%  (0.05104, 0.05835)
       a3 =    -0.02077;%  (-0.02263, -0.01891)
       b3 =      0.0122;%  (0.01082, 0.01357)
       a4 =     0.02649;%  (0.02237, 0.03062)
       b4 =    -0.01466;%  (-0.01829, -0.01102)
       a5 =    -0.06336;%  (-0.06658, -0.06014)
       b5 =    -0.01173;%  (-0.01711, -0.006361)
       a6 =    -0.01668;%  (-0.01775, -0.0156)
       b6 =   -0.008266;%  (-0.0106, -0.005927)
       a7 =   -0.008476;%  (-0.01065, -0.006302)
       b7 =    0.008158;%  (0.006576, 0.00974)
       w =     0.07714 ;% (0.07669, 0.0776)

syms x
f=a0 + a1*cos(x*w) + b1*sin(x*w)+a2*cos(2*x*w) + b2*sin(2*x*w) + a3*cos(3*x*w) + b3*sin(3*x*w) +  a4*cos(4*x*w) + b4*sin(4*x*w) + a5*cos(5*x*w) + b5*sin(5*x*w) +a6*cos(6*x*w) + b6*sin(6*x*w) + a7*cos(7*x*w) + b7*sin(7*x*w);
%figure
df=diff(f);
%fplot(df,[0 80])
%figure
df2=diff(f,2);
%fplot(df2,[0 80])
%figure
df3=diff(f,3);
%fplot(df3,[0 80])

f2=a02 + a12*cos(x*w2) + b12*sin(x*w2)+a22*cos(2*x*w2) + b22*sin(2*x*w2) + a32*cos(3*x*w2) + b32*sin(32*x*w2) +  a42*cos(4*x*w2) + b42*sin(4*x*w2) + a52*cos(5*x*w2) + b52*sin(5*x*w2) +a62*cos(6*x*w2) + b62*sin(6*x*w2) + a72*cos(7*x*w2) + b72*sin(7*x*w2);
df12=diff(f2)
df22=diff(f2,2)
df32=diff(f2,3)

m1=0.072;
m2=0.057;
lc1=0.11;
l1=0.22;
lc2=0.9;
l2=0.18;
iz1=0.000315866;
iz2=0.000159604;
g=9.81;
%syms A B D E F N
A=m1+(lc1^2)+iz1+m2*(l1^2)+m2*(lc2^2)+iz2+2*m2*l1*lc2*cos(f2);
B=m2*(lc2^2)+m2*l1*lc2*cos(f2);
D=2*m2*l1*lc2*sin(f2);
E=2*m2*l1*lc2*sin(f2);
F=m2*(lc2^2)+m2*l1*lc2*cos(f2)+iz2;
H=m2*(lc2^2)+iz2;
N=-m2*l1*lc2*sin(f2);

T1=A*df2+B*df22+D*df*df12+E*(df12^2)+g*((m1*lc1+m2*l1)*cos(f)+m2*lc2*cos(f+f2))
T2=F*df2+H*df22+N*df*df12+m2*l1*lc2*sin(f2)*df*(df+df12)+m2*g*lc2*cos(f+f2)
%figure
%fplot(T1,[0 80])
%figure
%fplot(T2,[0 80])
T=80
%RMS1 = sqrt(1/T*int(T1^2,0,T))
%RMS2 = sqrt(1/T*int(T1^2,0,T))




%aceleraciones
%motor 1
%(1472768451*cos((3857*x)/10000))/156250000000 - (39407713401*cos((3857*x)/12500))/15625000000000 - (37890315603*cos((3857*x)/25000))/31250000000000 + (55831313097*cos((11571*x)/25000))/15625000000000 - (65203475967*cos((3857*x)/50000))/25000000000000 + (278085461157*cos((11571*x)/50000))/250000000000000 + (3561689450819771643405817*cos((26999*x)/50000))/1441151880758558720000000000 + (17450074677*sin((3857*x)/10000))/10000000000000 + (10904437117*sin((3857*x)/12500))/7812500000000 - (8137417603*sin((3857*x)/25000))/6250000000000 + (637979806137584495056059*sin((11571*x)/25000))/360287970189639680000000000 - (6977054581*sin((3857*x)/50000))/6250000000000 - (8167170501*sin((11571*x)/50000))/12500000000000 - (3428063065100011926525329*sin((26999*x)/50000))/1441151880758558720000000000
%motor 2
%(5347840077*cos((981*x)/3125))/976562500000 - (97198461*cos((981*x)/2500))/7812500000 + (4939612833206717041563*cos((981*x)/6250))/45035996273704960000000000 + (47548100673713823197787*cos((2943*x)/6250))/720575940379279360000000000 + (5907934179*cos((981*x)/12500))/1562500000000 - (26564050683*cos((2943*x)/12500))/7812500000000 - (20700162112742456505273*cos((6867*x)/12500))/9007199254740992000000000 + (4472091567*sin((981*x)/2500))/625000000000 - (4123888809178509803169*sin((981*x)/3125))/45035996273704960000000000 - (5497704759330476794647*sin((981*x)/6250))/45035996273704960000000000 - (9761061073705322118057*sin((2943*x)/6250))/22517998136852480000000000 - (434024811*sin((981*x)/12500))/195312500000 - (11424187431*sin((2943*x)/12500))/3906250000000 + (294559365270751483404033*sin((6867*x)/12500))/360287970189639680000000000




function [x,y] = mediocirculo(cx,cy,r,n)
    a=linspace(5*pi/4,pi/4,n);
    x=cx+r*cos(a);
    y=cy+r*sin(a);

end
function [x,y] = octavocirculo(cx,cy,r,n)
    a=linspace(0,pi/4,n);
    x=cx+r*cos(a);
    y=cy+r*sin(a);

end
function [x,y] = octavocirculo2(cx,cy,r,n)
    a=linspace(5*pi/4,3*pi/2,n);
    x=cx+r*cos(a);
    y=cy+r*sin(a);

end