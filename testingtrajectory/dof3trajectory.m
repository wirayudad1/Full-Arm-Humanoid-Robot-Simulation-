close all;
clc;
clear all;

DEG2RAD = 0.01745329252;
RAD2DEG = 57.2958;
%Koordinat yang diinginkan
posx=[0,0,5,5];
posy=[6,12,6,12];
%iterasi
tmaks=24;
%panjang link
l1=9;
l2=6;
l3=3;

n=1;
%koordinat awal
X0 = 0;
Y0 = 0;
Z0 = 0;
%fungsi trajectory
while n<4
   xinit = posx(n);
   yinit = posy(n);
   xgoal = posx(n+1);
   ygoal = posy(n+1);
    for t=1:tmaks
    %persamaan work-Pos based Trajectory
    px=xinit+((xgoal-xinit)/tmaks*t);
    py=yinit+((ygoal-yinit)/tmaks*t);
    fprintf('nilai px(%d) = %d\n',t,px);
    %inverse kinematik
    sudutorientasi=atan(py/px);
    sudutorientasi=sudutorientasi*RAD2DEG;

    ybaru=py-(l3*sin(sudutorientasi*DEG2RAD));
    xbaru=px-(l3*cos(sudutorientasi*DEG2RAD));

    op=sqrt(xbaru^2+ybaru^2);
    t2= acos((op^2-l1^2-l2^2)/(2*l1*l2));
    t2=t2*RAD2DEG;%jika bernilai minus maka elbow down
    t1= atan(ybaru/xbaru)-atan((l2*sin(t2*DEG2RAD))/((op^2+l1^2-l2^2)/(2*l1)));
    t1=t1*RAD2DEG;
    t3=sudutorientasi-t1-t2;
    
    %Forward Kinematik
    % dof 1(rot Z) trans x
    RTZ_1 = [cos(t1*DEG2RAD) -sin(t1*DEG2RAD)  0 0; 
           sin(t1*DEG2RAD)  cos(t1*DEG2RAD)  0 0;
           0 0 1 0;
           0 0 0 1];
    TT1 = [1 0 0 l1; 
           0 1 0 0; 
           0 0 1 0; 
           0 0 0 1];
    FKT =  RTZ_1 * TT1;
    X1 = FKT(1, 4);
    Y1 = FKT(2, 4);
    Z1 = FKT(3, 4);
    %dof 2(rot Z) trans X
    RTZ_2 = [cos(t2*DEG2RAD) -sin(t2*DEG2RAD)  0 0; 
           sin(t2*DEG2RAD)  cos(t2*DEG2RAD)  0 0;
           0 0 1 0;
           0 0 0 1];
    TT2 = [1 0 0 l2; 
           0 1 0 0; 
           0 0 1 0; 
           0 0 0 1];
    FKT = FKT * RTZ_2 * TT2;
    X2 = FKT(1, 4);
    Y2 = FKT(2, 4);
    Z2 = FKT(3, 4);
    %dof 3(rot Z) trans X
    RTZ_3 = [cos(t3*DEG2RAD) -sin(t3*DEG2RAD)  0 0; 
           sin(t3*DEG2RAD)  cos(t3*DEG2RAD)  0 0;
           0 0 1 0;
           0 0 0 1];
    TT3 = [1 0 0 l3; 
           0 1 0 0; 
           0 0 1 0; 
           0 0 0 1];
    FKT = FKT * RTZ_3 * TT3;
    X3 = FKT(1, 4);
    Y3 = FKT(2, 4);
    Z3 = FKT(3, 4);
    %plot tangan robotic 3 dof planar
    figure(1);
    plot3([X0 X1], [Y0 Y1], [Z0 Z1], '-ko', ...
        [X1 X2], [Y1 Y2], [Z1 Z2], '-ko', ... 
        [X2 X3], [Y2 Y3], [Z2 Z3], '-ko', ... 
         'LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',7);    
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        axis([-10 20 -10 20 -10 20 ]);
        view(0, 90);
        grid on;
        pause(0.1);
    end
n=n+1;
end








