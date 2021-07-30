clear;
close all;
clc;

% Init Function -----------------------------------------------------------
link = [m2mm(0.0905),m2mm(0.0631),m2mm(0.0858),m2mm(0.056),m2mm(0.0354),m2mm(0.0652)];%->0,452
theta = [0,0,0,0,0,0,0];
%GoalPosition = [100,-50,100];%error
%GoalPosition = [50,0,-50];%aman
%GoalPosition = [-100,50,-100];
%GoalPosition = [100, -50,-200];%gerakan agak aneh
InitOr=[0,0,0,0,0,0];
  nameObject=char('bottle');
  %GoalOr=[88,6,67,71,69,68];%cerealbox
 GoalOr=[90,10,68,72,71,68];%bottle
  %GoalOr=[93,5,50,61,50,50];%glass
Heading = [0,0];
flag=0;
[~,~,InitPosition,~] = ForwardKinematics(theta,link);


vrep = remApi('remoteApi');
vrep.simxFinish(-1); % Just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

tic;
initime = cputime;
time1   = clock;
pause(1.0);  % Wait for a second;
fintime = cputime;
elapsed = toc;
time2   = clock;


if (clientID > -1)
    disp('Connected to remote API server');
    jointHandle = [0,0,0,0,0,0,0];
    objectHandle = 0;
    HandJoint = [0,0,0,0,0,0];
    [~, HandJoint(1)] = vrep.simxGetObjectHandle(clientID,'JJ1',vrep.simx_opmode_blocking);
    [~, HandJoint(2)] = vrep.simxGetObjectHandle(clientID,'JJ2',vrep.simx_opmode_blocking);
    [~, HandJoint(3)] = vrep.simxGetObjectHandle(clientID,'JJ3',vrep.simx_opmode_blocking);
    [~, HandJoint(4)] = vrep.simxGetObjectHandle(clientID,'JJ4',vrep.simx_opmode_blocking);
    [~, HandJoint(5)] = vrep.simxGetObjectHandle(clientID,'JJ5',vrep.simx_opmode_blocking);
    [~, HandJoint(6)] = vrep.simxGetObjectHandle(clientID,'JJ6',vrep.simx_opmode_blocking);
    
    [~, jointHandle(1)] = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_blocking);
    [~, jointHandle(2)] = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_blocking);
    [~, jointHandle(3)] = vrep.simxGetObjectHandle(clientID,'J3',vrep.simx_opmode_blocking);
    [~, jointHandle(4)] = vrep.simxGetObjectHandle(clientID,'J4',vrep.simx_opmode_blocking);
    [~, jointHandle(5)] = vrep.simxGetObjectHandle(clientID,'J5',vrep.simx_opmode_blocking);
    [~, jointHandle(6)] = vrep.simxGetObjectHandle(clientID,'J6',vrep.simx_opmode_blocking);
    [~, jointHandle(7)] = vrep.simxGetObjectHandle(clientID,'J7',vrep.simx_opmode_blocking);
    [~, objectHandle] = vrep.simxGetObjectHandle(clientID,nameObject,vrep.simx_opmode_blocking);
    
    
   % G = coordinateTransform([0,0,0,-90,90,0],GoalPosition);
   % G(1) = mm2m(G(1));
    %G(2) = mm2m(G(2));
    %G(3) = mm2m(G(3));
    
    %vrep.simxSetObjectPosition(clientID,objectHandle(1),jointHandle(2),[(G(1)) (G(2)) (G(3))],vrep.simx_opmode_blocking);
        
    [~, Goalgetvrep] = vrep.simxGetObjectPosition(clientID,objectHandle(1),jointHandle(2),vrep.simx_opmode_blocking); 
    Goalgetvrep = coordinateTransform([0,0,0,90,0,90],Goalgetvrep);
    Goalgetpos(1) = m2mm(Goalgetvrep(1));
    Goalgetpos(2) = m2mm(Goalgetvrep(2));
    Goalgetpos(3) = m2mm(Goalgetvrep(3));
    disp(Goalgetpos);
    %G = coordinateTransform([0,0,0,90,0,90],Goalgetpos);
   
    %[~,dimension]=vrep.simxGetObjectGroupData(clientID,objectHandle(1),vrep.simx_opmode_blocking);
   % disp(dimension);
    %initPosition Arm
    for i=1:7
        vrep.simxSetJointTargetPosition(clientID, jointHandle(i), deg2rad(theta(i)), vrep.simx_opmode_streaming);
    end
    for t=1:2
       if (Goalgetpos(2)>0)
          goalpos=[0,-30, Goalgetpos(3)+100];
       else
          goalpos=[0,Goalgetpos(2)-60, Goalgetpos(3)+100];
       end
       if(t==2)
           flag=1;
           InitPosition=goalpos;
           %goalpos=[Goalgetpos(1),Goalgetpos(2),Goalgetpos(3)];
           %goalpos=[Goalgetpos(1)-link(6),Goalgetpos(2),Goalgetpos(3)];
           %goalpos=[Goalgetpos(1)-link(6),Goalgetpos(2)-40,Goalgetpos(3)+30];
           if(Goalgetpos(2)>0)
               if(Goalgetpos(2)>70)
                   goalpos=[Goalgetpos(1)-link(6)-30,Goalgetpos(2)-80,Goalgetpos(3)+40];
               else
                   goalpos=[Goalgetpos(1)-link(6)-60,Goalgetpos(2)-50,Goalgetpos(3)+25];
               end
           else
               if(Goalgetpos(2)<-60)
                   goalpos=[Goalgetpos(1)-link(6)-70,Goalgetpos(2)-20,Goalgetpos(3)+40];
               else
                   goalpos=[Goalgetpos(1)-link(6)-60,Goalgetpos(2)-40,Goalgetpos(3)+40];
               end
           end
           
       end
       disp(goalpos);
      [tMax] = calculateIteration(InitPosition,goalpos,5);
    
    for ti=1:tMax
        P(1) = InitPosition(1) + ((goalpos(1)-InitPosition(1)) * ti / tMax);
        P(2) = InitPosition(2) + ((goalpos(2)-InitPosition(2)) * ti / tMax);
        P(3) = InitPosition(3) + ((goalpos(3)-InitPosition(3)) * ti / tMax);
         if(flag==1)
            Q(1,ti+tMax) = P(1);
            Q(2,ti+tMax) = P(2);
            Q(3,ti+tMax) = P(3);
        else
            Q(1,ti) = P(1);
            Q(2,ti) = P(2);
            Q(3,ti) = P(3);
        end
        [theta] = InverseKinematics(P,Heading,0,link);
        %theta(3)
        [Elbow,Forearm,Wrist,Palm,FKTARM] = ForwardKinematics(theta,link);
        DrawFKSimulation(Elbow,Forearm,Wrist,Palm,InitPosition,goalpos,[0,0,0,0,0,0],FKTARM);
        %drawHAND(FKTARM,Palm,[0,0,0,0,0,0]);
        for i=1:7
            vrep.simxSetJointTargetPosition(clientID, jointHandle(i), deg2rad(theta(i)), vrep.simx_opmode_streaming);
        end
    end
    end
    
    %hand trajectory
    [tMaxhand]=calculateIteration(InitOr,GoalOr,5);
    for ti=1:tMaxhand
        [tHand]=HandTraject(InitOr,GoalOr,ti,tMaxhand);
         DrawFKSimulation(Elbow,Forearm,Wrist,Palm,InitPosition,goalpos,tHand,FKTARM);
    for i=1:6
        vrep.simxSetJointTargetPosition(clientID, HandJoint(i), deg2rad(tHand(i)), vrep.simx_opmode_streaming);
    end
    disp(tHand);
    end
    %--
    
    drawpos(Q);
    [error]=err(P,Wrist);
    %disp(error);
    % End Init Function
    
else
    disp('Failed connecting to remote API server');
end
vrep.delete();              % call the destructor!
disp('Program ended');

function drawpos(Q)
    figure(2)
    subplot(3,1,1);
    plot(Q(1,:),'--r');
    ylabel('X position');
    grid on;
    subplot(3,1,2);
    plot(Q(2,:),'--g');
    ylabel('Y position');
    grid on;
    subplot(3,1,3);
    plot(Q(3,:),'--b');
    xlabel('Iteration');
    ylabel('Z position');
    grid on;
end
function [handTheta] = HandTraject(iT,gT,iter,t_max)
    handTheta(1)=iT(1)+((gT(1)-iT(1)))*iter/t_max;
    handTheta(2)=iT(2)+((gT(2)-iT(2)))*iter/t_max;
    handTheta(3)=iT(3)+((gT(3)-iT(3)))*iter/t_max;
    handTheta(4)=iT(4)+((gT(4)-iT(4)))*iter/t_max;
    handTheta(5)=iT(5)+((gT(5)-iT(5)))*iter/t_max;
    handTheta(6)=iT(6)+((gT(6)-iT(6)))*iter/t_max;
end
function [error] = err(GoalPos,EoF)
  error(1) = ((abs(EoF(1)-GoalPos(1))/abs(GoalPos(1))))*100;
  error(2) = ((abs(EoF(2)-GoalPos(2))/abs(GoalPos(2))))*100;
  error(3) = ((abs(EoF(3)-GoalPos(3))/abs(GoalPos(3))))*100;
  error=(error(1)+error(2)+error(3))/3;
end
function [mm] = m2mm(m)
    mm = m*1000;
end

function [m] = mm2m(mm)
    m = mm/1000;
end

function DrawFKSimulation(elbow,forearm,wrist,palm,I,G,theta,FKTARM)

   %Base
% Translation Z
TZ_1 = [        1              0               0               0 ;
                0              1               0               0 ;
                0              0               1            -22.78; 
                0              0               0               1 ];
FKB = FKTARM*TZ_1;
X1 = FKB(1, 4);
Y1 = FKB(2, 4);
Z1 = FKB(3, 4);

% Thumb Finger------------------------------------------------------------
% Virtual Rotation Y - Thumb 1A
VRY_T1A = [cos(deg2rad(-43.88))          0       sin(deg2rad(-43.88))        0;
                   0                    1               0                  0;
            -sin(deg2rad(-43.88))        0       cos(deg2rad(-43.88))        0; 
                   0                    0               0                  1 ];

% Translation Z               
TY_T1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1            -49.28; 
                 0              0               0               1 ];

% Virtual Rotation Y - Thumb 1B
VRY_T1B = [cos(deg2rad(43.88))         0        sin(deg2rad(43.88))        0;
                   0                  1                0                 0;
           -sin(deg2rad(43.88))        0        cos(deg2rad(43.88))        0;
                   0                  0                0                 1 ];
               
FKTT = FKB * VRY_T1A * TY_T1 * VRY_T1B;
XT1 = FKTT(1, 4);
YT1 = FKTT(2, 4);
ZT1 = FKTT(3, 4);

% Rotation Z
RZ_T1 = [cos(deg2rad(theta(1))) -sin(deg2rad(theta(1)))      0               0;
         sin(deg2rad(theta(1)))  cos(deg2rad(theta(1)))      0               0;
                0                 0              1               0;
                0                 0              0               1 ];

% Translation X
TX_T2 = [        1              0               0             38.48;
                 0              1               0               0 ;
                 0              0               1               0 ; 
                 0              0               0               1 ];
               
FKTT = FKTT * RZ_T1 * TX_T2;
XT2 = FKTT(1, 4);
YT2 = FKTT(2, 4);
ZT2 = FKTT(3, 4);

% Rotation Y
RY_T2 = [cos(deg2rad(25+theta(2)))        0        sin(deg2rad(25+theta(2)))        0;
                  0                   1               0                     0;
         -sin(deg2rad(25+theta(2)))       0        cos(deg2rad(25+theta(2)))        0; 
                  0                   0               0                     1 ];

% Translation X
TX_T3 = [         1                 0               0               40.5;
                  0                 1               0                0;
                  0                 0               1                0; 
                  0                 0               0                1 ];

FKTT = FKTT * RY_T2 * TX_T3;
XT3 = FKTT(1, 4);
YT3 = FKTT(2, 4);
ZT3 = FKTT(3, 4);
              
VRY_T2A = [cos(deg2rad(32.63))        0        sin(deg2rad(32.63))        0;
                  0                    1               0                    0;
           -sin(deg2rad(32.63))       0        cos(deg2rad(32.63))        0; 
                  0                    0               0                    1 ];

TX_T4 = [       1                 0               0               45.25;
                  0                 1               0                0;
                  0                 0               1                0; 
                  0                 0               0                1 ];

VRY_T2B = [cos(deg2rad(-32.63))        0        sin(deg2rad(-32.63))       0;
                    0                     1               0                    0;
             -sin(deg2rad(-32.63))       0        cos(deg2rad(-32.63))       0; 
                    0                     0               0                    1 ];
                
FKTT = FKTT * VRY_T2A * TX_T4 * VRY_T2B;
XT4 = FKTT(1, 4);
YT4 = FKTT(2, 4);
ZT4 = FKTT(3, 4);
% Thumb Finger------------------------------------------------------------


%Index Finger------------------------------------------------------------
% |
VRY_I1A = [cos(deg2rad(-17))        0        sin(deg2rad(-17))        0;
                 0                 1               0                0;
           -sin(deg2rad(-17))       0        cos(deg2rad(-17))        0; 
                 0                 0               0                1 ];
               
TY_I1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -119; 
                 0              0               0               1 ];
            
VRY_I1B = [cos(deg2rad(17))       0        sin(deg2rad(17))         0;
                 0               1              0                 0;
           -sin(deg2rad(17))      0        cos(deg2rad(17))         0;
                 0               0              0                 1 ];
               
FKIT = FKB * VRY_I1A * TY_I1 * VRY_I1B;
XI1 = FKIT(1, 4);
YI1 = FKIT(2, 4);
ZI1 = FKIT(3, 4);


VRY_I2A = [cos(deg2rad(-3))        0        sin(deg2rad(-3))        0;
                 0                1              0                0;
           -sin(deg2rad(-3))       0        cos(deg2rad(-3))        0; 
                 0                0              0                1 ];

RX_I1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(3))) -sin(deg2rad(25+theta(3)))    0;
               0        sin(deg2rad(25+theta(3)))  cos(deg2rad(25+theta(3)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_I2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKIT = FKIT * VRY_I2A * RX_I1 * TY_I2;
XI2 = FKIT(1, 4);
YI2 = FKIT(2, 4);
ZI2 = FKIT(3, 4);

VRX_I3A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_I3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_I3B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKIT = FKIT * VRX_I3A * TY_I3 * VRX_I3B;
XI3 = FKIT(1, 4);
YI3 = FKIT(2, 4);
ZI3 = FKIT(3, 4);
% |
%Index Finger------------------------------------------------------------


%Middle Finger------------------------------------------------------------
% |
VRY_M1A = [cos(deg2rad(-5))        0        sin(deg2rad(-5))        0;
                 0                 1               0                0;
           -sin(deg2rad(-5))       0        cos(deg2rad(-5))        0; 
                 0                 0               0                1 ];
               
TY_M1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -117; 
                 0              0               0               1 ];
            
VRY_M1B = [cos(deg2rad(5))       0        sin(deg2rad(5))         0;
                 0               1              0                 0;
           -sin(deg2rad(5))      0        cos(deg2rad(5))         0;
                 0               0              0                 1 ];
               
FKMT = FKB * VRY_M1A * TY_M1 * VRY_M1B;
XM1 = FKMT(1, 4);
YM1 = FKMT(2, 4);
ZM1 = FKMT(3, 4);

RX_M1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(4))) -sin(deg2rad(25+theta(4)))    0;
               0        sin(deg2rad(25+theta(4)))  cos(deg2rad(25+theta(4)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_M2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKMT = FKMT * RX_M1 * TY_M2;
XM2 = FKMT(1, 4);
YM2 = FKMT(2, 4);
ZM2 = FKMT(3, 4);

VRX_M2A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_M3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_M2B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKMT = FKMT * VRX_M2A * TY_M3 * VRX_M2B;
XM3 = FKMT(1, 4);
YM3 = FKMT(2, 4);
ZM3 = FKMT(3, 4);
% |
% Middle Finger -----------------------------------------------------------


%Ring Finger------------------------------------------------------------
% |
VRY_R1A = [cos(deg2rad(7))        0        sin(deg2rad(7))        0;
               0                 1               0              0;
           -sin(deg2rad(7))       0        cos(deg2rad(7))        0; 
               0                 0               0              1 ];
               
TY_R1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -113; 
                 0              0               0               1 ];
            
VRY_R1B = [cos(deg2rad(-7))       0        sin(deg2rad(-7))         0;
                 0               1              0                 0;
           -sin(deg2rad(-7))      0        cos(deg2rad(-7))         0;
                 0               0              0                 1 ];
               
FKRT = FKB * VRY_R1A * TY_R1 * VRY_R1B;
XR1 = FKRT(1, 4);
YR1 = FKRT(2, 4);
ZR1 = FKRT(3, 4);


VRY_R2A = [cos(deg2rad(3))        0        sin(deg2rad(3))        0;
                 0                1              0                0;
           -sin(deg2rad(3))       0        cos(deg2rad(3))        0; 
                 0                0              0                1 ];

RX_R1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(5))) -sin(deg2rad(25+theta(5)))    0;
               0        sin(deg2rad(25+theta(5)))  cos(deg2rad(25+theta(5)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_R2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKRT = FKRT * VRY_R2A * RX_R1 * TY_R2;
XR2 = FKRT(1, 4);
YR2 = FKRT(2, 4);
ZR2 = FKRT(3, 4);

VRX_R3A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_R3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_R3B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKRT = FKRT * VRX_R3A * TY_R3 * VRX_R3B;
XR3 = FKRT(1, 4);
YR3 = FKRT(2, 4);
ZR3 = FKRT(3, 4);
%Ring Finger------------------------------------------------------------

%Little Finger------------------------------------------------------------
% |
VRY_L1A = [cos(deg2rad(13))        0        sin(deg2rad(13))        0;
               0                 1               0              0;
           -sin(deg2rad(13))       0        cos(deg2rad(13))        0; 
               0                 0               0              1 ];
               
TY_L1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -109; 
                 0              0               0               1 ];
            
VRY_L1B = [cos(deg2rad(-13))       0        sin(deg2rad(-13))         0;
                 0               1              0                 0;
           -sin(deg2rad(-13))      0        cos(deg2rad(-13))         0;
                 0               0              0                 1 ];
               
FKLT = FKB * VRY_L1A * TY_L1 * VRY_L1B;
XL1 = FKLT(1, 4);
YL1 = FKLT(2, 4);
ZL1 = FKLT(3, 4);


VRY_L2A = [cos(deg2rad(3))        0        sin(deg2rad(3))        0;
                 0                1              0                0;
           -sin(deg2rad(3))       0        cos(deg2rad(3))        0; 
                 0                0              0                1 ];

RX_L1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(6))) -sin(deg2rad(25+theta(6)))    0;
               0        sin(deg2rad(25+theta(6)))  cos(deg2rad(25+theta(6)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_L2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKLT = FKLT * VRY_L2A * RX_L1 * TY_L2;
XL2 = FKLT(1, 4);
YL2 = FKLT(2, 4);
ZL2 = FKLT(3, 4);

VRX_L3A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_L3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_L3B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKLT = FKLT * VRX_R3A * TY_R3 * VRX_R3B;
XL3 = FKLT(1, 4);
YL3 = FKLT(2, 4);
ZL3 = FKLT(3, 4);
%Ring Finger------------------------------------------------------------

    plot3([0 95], [0 0], [0 0], '-k<', ...
        [0 0], [0 95], [0 0], '-k>', ...
        [0 0], [0 0], [0 -95], '-kv', ...
        [0 elbow(1)], [0 elbow(2)], [0 elbow(3)], '-ko', ...
        [elbow(1) forearm(1)], [elbow(2) forearm(2)], [elbow(3) forearm(3)], '-ko', ...
        [forearm(1) wrist(1)], [forearm(2) wrist(2)], [forearm(3) wrist(3)], '-ko', ...
        [wrist(1) X1], [wrist(2) Y1], [wrist(3) Z1], '-ko', ...
        [X1 XT1], [Y1 YT1], [Z1 ZT1], '-ko', ...
        [XT1 XT2], [YT1 YT2], [ZT1 ZT2], '-ko', ...
        [XT2 XT3], [YT2 YT3], [ZT2 ZT3], '-ko', ...
        [XT3 XT4], [YT3 YT4], [ZT3 ZT4], '-ko', ...
        [X1 XI1], [Y1 YI1], [Z1 ZI1], '-ko', ...
        [XI1 XI2], [YI1 YI2], [ZI1 ZI2], '-ko', ...
        [XI2 XI3], [YI2 YI3], [ZI2 ZI3], '-ko', ...
        [X1 XM1], [Y1 YM1], [Z1 ZM1], '-ko', ...
        [XM1 XM2], [YM1 YM2], [ZM1 ZM2], '-ko', ...
        [XM2 XM3], [YM2 YM3], [ZM2 ZM3], '-ko', ...
        [X1 XR1], [Y1 YR1], [Z1 ZR1], '-ko', ...
        [XR1 XR2], [YR1 YR2], [ZR1 ZR2], '-ko', ...
        [XR2 XR3], [YR2 YR3], [ZR2 ZR3], '-ko', ...
        [X1 XL1], [Y1 YL1], [Z1 ZL1], '-ko', ...
        [XL1 XL2], [YL1 YL2], [ZL1 ZL2], '-ko', ...
        [XL2 XL3], [YL2 YL3], [ZL2 ZL3], '-ko', ...
        [I(1) G(1)], [I(2) G(2)], [I(3) G(3)], '--ko', ...
        'LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',7);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis([-200 500 -700 300 -950 300]);
    view(90, 30); 
    grid on;
    pause(0.1);
end
function drawHAND(FKTARM,palm,theta)
      %Base
% Translation Z
TZ_1 = [        1              0               0               0 ;
                0              1               0               0 ;
                0              0               1            0.78; 
                0              0               0               1 ];
FKB = FKTARM*TZ_1;
X1 = FKB(1, 4);
Y1 = FKB(2, 4);
Z1 = FKB(3, 4);

% Thumb Finger------------------------------------------------------------
% Virtual Rotation Y - Thumb 1A
VRY_T1A = [cos(deg2rad(-43.88))          0       sin(deg2rad(-43.88))        0;
                   0                    1               0                  0;
            -sin(deg2rad(-43.88))        0       cos(deg2rad(-43.88))        0; 
                   0                    0               0                  1 ];

% Translation Z               
TY_T1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1            -49.28; 
                 0              0               0               1 ];

% Virtual Rotation Y - Thumb 1B
VRY_T1B = [cos(deg2rad(43.88))         0        sin(deg2rad(43.88))        0;
                   0                  1                0                 0;
           -sin(deg2rad(43.88))        0        cos(deg2rad(43.88))        0;
                   0                  0                0                 1 ];
               
FKTT = FKB * VRY_T1A * TY_T1 * VRY_T1B;
XT1 = FKTT(1, 4);
YT1 = FKTT(2, 4);
ZT1 = FKTT(3, 4);

% Rotation Z
RZ_T1 = [cos(deg2rad(theta(1))) -sin(deg2rad(theta(1)))      0               0;
         sin(deg2rad(theta(1)))  cos(deg2rad(theta(1)))      0               0;
                0                 0              1               0;
                0                 0              0               1 ];

% Translation X
TX_T2 = [        1              0               0             38.48;
                 0              1               0               0 ;
                 0              0               1               0 ; 
                 0              0               0               1 ];
               
FKTT = FKTT * RZ_T1 * TX_T2;
XT2 = FKTT(1, 4);
YT2 = FKTT(2, 4);
ZT2 = FKTT(3, 4);

% Rotation Y
RY_T2 = [cos(deg2rad(25+theta(2)))        0        sin(deg2rad(25+theta(2)))        0;
                  0                   1               0                     0;
         -sin(deg2rad(25+theta(2)))       0        cos(deg2rad(25+theta(2)))        0; 
                  0                   0               0                     1 ];

% Translation X
TX_T3 = [         1                 0               0               40.5;
                  0                 1               0                0;
                  0                 0               1                0; 
                  0                 0               0                1 ];

FKTT = FKTT * RY_T2 * TX_T3;
XT3 = FKTT(1, 4);
YT3 = FKTT(2, 4);
ZT3 = FKTT(3, 4);
              
VRY_T2A = [cos(deg2rad(32.63))        0        sin(deg2rad(32.63))        0;
                  0                    1               0                    0;
           -sin(deg2rad(32.63))       0        cos(deg2rad(32.63))        0; 
                  0                    0               0                    1 ];

TX_T4 = [       1                 0               0               45.25;
                  0                 1               0                0;
                  0                 0               1                0; 
                  0                 0               0                1 ];

VRY_T2B = [cos(deg2rad(-32.63))        0        sin(deg2rad(-32.63))       0;
                    0                     1               0                    0;
             -sin(deg2rad(-32.63))       0        cos(deg2rad(-32.63))       0; 
                    0                     0               0                    1 ];
                
FKTT = FKTT * VRY_T2A * TX_T4 * VRY_T2B;
XT4 = FKTT(1, 4);
YT4 = FKTT(2, 4);
ZT4 = FKTT(3, 4);
% Thumb Finger------------------------------------------------------------


%Index Finger------------------------------------------------------------
% |
VRY_I1A = [cos(deg2rad(-17))        0        sin(deg2rad(-17))        0;
                 0                 1               0                0;
           -sin(deg2rad(-17))       0        cos(deg2rad(-17))        0; 
                 0                 0               0                1 ];
               
TY_I1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -119; 
                 0              0               0               1 ];
            
VRY_I1B = [cos(deg2rad(17))       0        sin(deg2rad(17))         0;
                 0               1              0                 0;
           -sin(deg2rad(17))      0        cos(deg2rad(17))         0;
                 0               0              0                 1 ];
               
FKIT = FKB * VRY_I1A * TY_I1 * VRY_I1B;
XI1 = FKIT(1, 4);
YI1 = FKIT(2, 4);
ZI1 = FKIT(3, 4);


VRY_I2A = [cos(deg2rad(-3))        0        sin(deg2rad(-3))        0;
                 0                1              0                0;
           -sin(deg2rad(-3))       0        cos(deg2rad(-3))        0; 
                 0                0              0                1 ];

RX_I1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(3))) -sin(deg2rad(25+theta(3)))    0;
               0        sin(deg2rad(25+theta(3)))  cos(deg2rad(25+theta(3)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_I2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKIT = FKIT * VRY_I2A * RX_I1 * TY_I2;
XI2 = FKIT(1, 4);
YI2 = FKIT(2, 4);
ZI2 = FKIT(3, 4);

VRX_I3A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_I3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_I3B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKIT = FKIT * VRX_I3A * TY_I3 * VRX_I3B;
XI3 = FKIT(1, 4);
YI3 = FKIT(2, 4);
ZI3 = FKIT(3, 4);
% |
%Index Finger------------------------------------------------------------


%Middle Finger------------------------------------------------------------
% |
VRY_M1A = [cos(deg2rad(-5))        0        sin(deg2rad(-5))        0;
                 0                 1               0                0;
           -sin(deg2rad(-5))       0        cos(deg2rad(-5))        0; 
                 0                 0               0                1 ];
               
TY_M1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -117; 
                 0              0               0               1 ];
            
VRY_M1B = [cos(deg2rad(5))       0        sin(deg2rad(5))         0;
                 0               1              0                 0;
           -sin(deg2rad(5))      0        cos(deg2rad(5))         0;
                 0               0              0                 1 ];
               
FKMT = FKB * VRY_M1A * TY_M1 * VRY_M1B;
XM1 = FKMT(1, 4);
YM1 = FKMT(2, 4);
ZM1 = FKMT(3, 4);

RX_M1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(4))) -sin(deg2rad(25+theta(4)))    0;
               0        sin(deg2rad(25+theta(4)))  cos(deg2rad(25+theta(4)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_M2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKMT = FKMT * RX_M1 * TY_M2;
XM2 = FKMT(1, 4);
YM2 = FKMT(2, 4);
ZM2 = FKMT(3, 4);

VRX_M2A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_M3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_M2B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKMT = FKMT * VRX_M2A * TY_M3 * VRX_M2B;
XM3 = FKMT(1, 4);
YM3 = FKMT(2, 4);
ZM3 = FKMT(3, 4);
% |
% Middle Finger -----------------------------------------------------------


%Ring Finger------------------------------------------------------------
% |
VRY_R1A = [cos(deg2rad(7))        0        sin(deg2rad(7))        0;
               0                 1               0              0;
           -sin(deg2rad(7))       0        cos(deg2rad(7))        0; 
               0                 0               0              1 ];
               
TY_R1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -113; 
                 0              0               0               1 ];
            
VRY_R1B = [cos(deg2rad(-7))       0        sin(deg2rad(-7))         0;
                 0               1              0                 0;
           -sin(deg2rad(-7))      0        cos(deg2rad(-7))         0;
                 0               0              0                 1 ];
               
FKRT = FKB * VRY_R1A * TY_R1 * VRY_R1B;
XR1 = FKRT(1, 4);
YR1 = FKRT(2, 4);
ZR1 = FKRT(3, 4);


VRY_R2A = [cos(deg2rad(3))        0        sin(deg2rad(3))        0;
                 0                1              0                0;
           -sin(deg2rad(3))       0        cos(deg2rad(3))        0; 
                 0                0              0                1 ];

RX_R1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(5))) -sin(deg2rad(25+theta(5)))    0;
               0        sin(deg2rad(25+theta(5)))  cos(deg2rad(25+theta(5)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_R2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKRT = FKRT * VRY_R2A * RX_R1 * TY_R2;
XR2 = FKRT(1, 4);
YR2 = FKRT(2, 4);
ZR2 = FKRT(3, 4);

VRX_R3A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_R3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_R3B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKRT = FKRT * VRX_R3A * TY_R3 * VRX_R3B;
XR3 = FKRT(1, 4);
YR3 = FKRT(2, 4);
ZR3 = FKRT(3, 4);
%Ring Finger------------------------------------------------------------

%Little Finger------------------------------------------------------------
% |
VRY_L1A = [cos(deg2rad(13))        0        sin(deg2rad(13))        0;
               0                 1               0              0;
           -sin(deg2rad(13))       0        cos(deg2rad(13))        0; 
               0                 0               0              1 ];
               
TY_L1 = [        1              0               0               0 ;
                 0              1               0               0 ;
                 0              0               1             -109; 
                 0              0               0               1 ];
            
VRY_L1B = [cos(deg2rad(-13))       0        sin(deg2rad(-13))         0;
                 0               1              0                 0;
           -sin(deg2rad(-13))      0        cos(deg2rad(-13))         0;
                 0               0              0                 1 ];
               
FKLT = FKB * VRY_L1A * TY_L1 * VRY_L1B;
XL1 = FKLT(1, 4);
YL1 = FKLT(2, 4);
ZL1 = FKLT(3, 4);


VRY_L2A = [cos(deg2rad(3))        0        sin(deg2rad(3))        0;
                 0                1              0                0;
           -sin(deg2rad(3))       0        cos(deg2rad(3))        0; 
                 0                0              0                1 ];

RX_L1 = [      1                 0                    0               0;
               0        cos(deg2rad(25+theta(6))) -sin(deg2rad(25+theta(6)))    0;
               0        sin(deg2rad(25+theta(6)))  cos(deg2rad(25+theta(6)))    0;
               0                 0                    0               1 ];

% Translation Z
TY_L2 = [         1                 0               0                0;
                  0                 1               0                0;
                  0                 0               1              -40.5; 
                  0                 0               0                1 ];
              
FKLT = FKLT * VRY_L2A * RX_L1 * TY_L2;
XL2 = FKLT(1, 4);
YL2 = FKLT(2, 4);
ZL2 = FKLT(3, 4);

VRX_L3A = [       1                 0               0                0;
                  0      cos(deg2rad(32.63)) -sin(deg2rad(32.63))  0;
                  0      sin(deg2rad(32.63)) cos(deg2rad(32.63))   0;
                  0                 0               0                1 ];
              
TY_L3 = [        1                 0               0                 0;
                 0                 1               0                 0;
                 0                 0               1              -45.25; 
                 0                 0               0                 1 ];

VRX_L3B = [       1                 0               0                  0;
                  0      cos(deg2rad(-32.63)) -sin(deg2rad(-32.63))  0;
                  0      sin(deg2rad(-32.63)) cos(deg2rad(-32.63))   0; 
                  0                 0               0                  1 ];

FKLT = FKLT * VRX_R3A * TY_R3 * VRX_R3B;
XL3 = FKLT(1, 4);
YL3 = FKLT(2, 4);
ZL3 = FKLT(3, 4);
%Ring Finger------------------------------------------------------------
end

function [objectPosRefOrgB] = coordinateTransform(poseOrgA2OrgB,objectPosRefOrgA)
    RX = [1 0 0 0; 0 cos(deg2rad(poseOrgA2OrgB(4))) -sin(deg2rad(poseOrgA2OrgB(4))) 0; 0 sin(deg2rad(poseOrgA2OrgB(4))) cos(deg2rad(poseOrgA2OrgB(4))) 0; 0 0 0 1];
    RY = [cos(deg2rad(poseOrgA2OrgB(5))) 0 sin(deg2rad(poseOrgA2OrgB(5))) 0; 0 1 0 0; -sin(deg2rad(poseOrgA2OrgB(5))) 0 cos(deg2rad(poseOrgA2OrgB(5))) 0; 0 0 0 1];
    RZ = [cos(deg2rad(poseOrgA2OrgB(6))) -sin(deg2rad(poseOrgA2OrgB(6))) 0 0; sin(deg2rad(poseOrgA2OrgB(6))) cos(deg2rad(poseOrgA2OrgB(6))) 0 0; 0 0 1 0; 0 0 0 1];
    TT = [1 0 0 poseOrgA2OrgB(1); 0 1 0 poseOrgA2OrgB(2); 0 0 1 poseOrgA2OrgB(3); 0 0 0 1];
    tempMat = (RX * RY * RZ * TT) \ [0,0,0,objectPosRefOrgA(1); 0,0,0,objectPosRefOrgA(2); 0,0,0,objectPosRefOrgA(3); 0,0,0,1];
    objectPosRefOrgB = [tempMat(1,4),tempMat(2,4),tempMat(3,4)];
end

function [loop] = calculateIteration(iP,gP,d) %initPos, goalPos, distance
    R = sqrt(((iP(1)-gP(1))^2+(iP(2)-gP(2))^2+(iP(3)-gP(3))^2));
    loop = floor(R/d);
end

function [elbow,forearm,wrist,palm,FKT] = ForwardKinematics(theta,link)
    %FK Wrist
    RY_1 = [cos(deg2rad(theta(1)))      0        sin(deg2rad(theta(1)))     0;
                   0                1               0               0;
            -sin(deg2rad(theta(1)))     0        cos(deg2rad(theta(1)))     0; 
                   0                0               0               1 ];
    FKT = RY_1;

    RX_2 = [       1               0               0               0;
                   0     cos(deg2rad(theta(2))) -sin(deg2rad(theta(2)))    0;
                   0     sin(deg2rad(theta(2)))  cos(deg2rad(theta(2)))    0;
                   0                0              0               1 ];
    TZ_1 = [        1              0               0               0 ;
                    0              1               0               0 ;
                    0              0               1              -link(1); 
                    0              0               0               1 ];
    FKT = FKT * RX_2 * TZ_1;

    RZ_3 = [cos(deg2rad(theta(3))) -sin(deg2rad(theta(3)))     0            0;
            sin(deg2rad(theta(3)))  cos(deg2rad(theta(3)))     0            0;
                   0                0              1               0;
                   0                0              0               1 ];
    TZ_2 = [        1              0               0               0 ;
                    0              1               0               0 ;
                    0              0               1              -link(2); 
                    0              0               0               1 ];            
    FKT = FKT * RZ_3 * TZ_2;

    elbow(1) = (-(link(1)+link(2))) * cos(deg2rad(theta(2))) * sin(deg2rad(theta(1)));
    elbow(2) = (link(1)+link(2)) * sin(deg2rad(theta(2)));
    elbow(3) = (-(link(1)+link(2))) * cos(deg2rad(theta(1))) * cos(deg2rad(theta(2)));

    RY_4 = [cos(deg2rad(theta(4)))       0        sin(deg2rad(theta(4)))       0;
                   0               1               0               0;
            -sin(deg2rad(theta(4)))      0        cos(deg2rad(theta(4)))       0; 
                   0               0               0               1 ];
    TZ_3 = [       1               0               0               0 ;
                   0               1               0               0 ;
                   0               0               1              -link(3); 
                   0               0               0               1 ];
    FKT = FKT * RY_4 * TZ_3;

    % DoF-5 Yaw-2
    RZ_5 = [cos(deg2rad(theta(5))) -sin(deg2rad(theta(5)))      0               0;
            sin(deg2rad(theta(5)))  cos(deg2rad(theta(5)))      0               0;
                   0                 0              1               0;
                   0                 0              0               1 ];
    % Link 4
    TZ_4 = [        1              0               0               0 ;
                    0              1               0               0 ;
                    0              0               1              -link(4); 
                    0              0               0               1 ];

    FKT = FKT * RZ_5 * TZ_4;
    forearm(1) = FKT(1, 4);
    forearm(2) = FKT(2, 4);
    forearm(3) = FKT(3, 4);

    % DoF-6 Pitch-3
    RY_6 = [cos(deg2rad(theta(6)))       0        sin(deg2rad(theta(6)))     0;
                 0                 1               0             0;
            -sin(deg2rad(theta(6)))      0        cos(deg2rad(theta(6)))     0; 
                   0               0               0             1 ];

    TZ_5 = [        1              0               0               0 ;
                    0              1               0               0 ;
                    0              0               1              -link(5); 
                    0              0               0               1 ];        
    FKT = FKT * RY_6 * TZ_5;
    wrist(1) = FKT(1, 4);
    wrist(2) = FKT(2, 4);
    wrist(3) = FKT(3, 4);

    RX_7 = [       1               0               0               0;
                   0        cos(deg2rad(theta(7))) -sin(deg2rad(theta(7)))     0;
                   0        sin(deg2rad(theta(7)))  cos(deg2rad(theta(7)))     0;
                   0                0              0               1 ];
    TZ_7 = [        1              0               0               0 ;
                    0              1               0               0 ;
                    0              0               1              -link(6); 
                    0              0               0               1 ];
    FKT = FKT * RX_7 * TZ_7;

    palm(1) = FKT(1, 4);
    palm(2) = FKT(2, 4);
    palm(3) = FKT(3, 4);
end

function [t] = InverseKinematics(P,H,RPY,l)
    %Transversal
    tr(1) = rad2deg(atan2(P(2), P(1)))/2;
    B = rad2deg(atan2(P(2), P(1))) - tr(1);
    px = (sqrt((P(1)^2) + (P(2)^2))) * cos(deg2rad(B));
    py = (sqrt((P(1)^2) + (P(2)^2))) * sin(deg2rad(B));
    %Frontal
    R(1) = sqrt(((-P(3))^2) + (py^2));
    tr(2) = rad2deg(atan2(py,(-P(3))));
    %Sagittal
    %Sagittal
    tr(4) = -rad2deg(acos(((-(-R(1)))^2+(-px)^2-(l(1)+l(2))^2-(l(3)+l(4)+l(5))^2)/(2*(l(1)+l(2))*(l(3)+l(4)+l(5))))); 
    tr(3) = rad2deg((atan2((-px),(-(-R(1)))))-(atan2((l(3)+l(4)+l(5)) * sin(deg2rad(tr(4))),((l(1)+l(2)) + ((l(3)+l(4)+l(5)) * cos(deg2rad(tr(4))))))));
    %tr(4) = -rad2deg(acos((R(1)^2+(-px)^2-(l(1)+l(2))^2-(l(3)+l(4)+l(5))^2)/(2*(l(1)+l(2))*(l(3)+l(4)+l(5))))); %(-px) karena sumbu X pada solusi invers mengarah ke -
    %tr(3) = rad2deg((atan2((-px),R(1)))-(atan2((l(3)+l(4)+l(5)) * sin(deg2rad(tr(4))),((l(1)+l(2)) + ((l(3)+l(4)+l(5)) * cos(deg2rad(tr(4)))))))); %(-px) karena sumbu X pada solusi invers mengarah ke -
 

    %FK ELBOW REF
    E_ElbowX = -(l(1)+l(2)) * (cos(deg2rad(tr(1))) * sin(deg2rad(tr(3))) + cos(deg2rad(tr(3))) * sin(deg2rad(tr(1))) * sin(deg2rad(tr(2))));
    E_ElbowY = (l(1)+l(2)) * ((cos(deg2rad(tr(1))) * cos(deg2rad(tr(3))) * sin(deg2rad(tr(2)))) + (-sin(deg2rad(tr(1))) * sin(deg2rad(tr(3)))));
    E_ElbowZ = (-(l(1)+l(2)) * cos(deg2rad(tr(2))) * cos(deg2rad(tr(3))));
    
    %FK WRIST REF
    E_WristX = (-(l(3)+l(4)+l(5)) * cos(deg2rad(tr(4))) * (cos(deg2rad(tr(1))) * sin(deg2rad(tr(3))) + cos(deg2rad(tr(3))) * sin(deg2rad(tr(1))) * sin(deg2rad(tr(2))))) + (-(l(3)+l(4)+l(5)) * sin(deg2rad(tr(4))) * (cos(deg2rad(tr(1))) * cos(deg2rad(tr(3)))-sin(deg2rad(tr(1)))*sin(deg2rad(tr(2)))*sin(deg2rad(tr(3))))) + (-(l(1)+l(2)) * cos(deg2rad(tr(1))) * sin(deg2rad(tr(3)))) + (-(l(1)+l(2)) * cos(deg2rad(tr(3))) * sin(deg2rad(tr(1))) * sin(deg2rad(tr(2))));
    E_WristY = ((l(1)+l(2)) * cos(deg2rad(tr(1))) * cos(deg2rad(tr(3))) * sin(deg2rad(tr(2)))) + (-(l(3)+l(4)+l(5)) * sin(deg2rad(tr(4))) * (cos(deg2rad(tr(3))) * sin(deg2rad(tr(1))) + cos(deg2rad(tr(1))) * sin(deg2rad(tr(2))) * sin(deg2rad(tr(3))))) + (-(l(1)+l(2)) * sin(deg2rad(tr(1))) * sin(deg2rad(tr(3)))) + (-(l(3)+l(4)+l(5)) * cos(deg2rad(tr(4))) * (sin(deg2rad(tr(1))) * sin(deg2rad(tr(3))) - cos(deg2rad(tr(1))) * cos(deg2rad(tr(3))) * sin(deg2rad(tr(2)))));
    E_WristZ = cos(deg2rad(tr(2))) * ((l(3)+l(4)+l(5)) * sin(deg2rad(tr(3))) * sin(deg2rad(tr(4))) - (l(3)+l(4)+l(5)) * cos(deg2rad(tr(3))) * cos(deg2rad(tr(4))) - (l(1)+l(2)) * cos(deg2rad(tr(3))));
    
    %Recalculated Sagittal
    t(1) = rad2deg(atan2((-E_ElbowX),(-E_ElbowZ)));
    t(4) = tr(4);
    %Recalculated Frontal
    t(2) = rad2deg(asin(E_ElbowY/(l(1)+l(2))));
    
    %FK ELBOW REAL
    %W_ElbowX = (-(l(1)+l(2))) * cos(deg2rad(t2)) * sin(deg2rad(t1));
    %W_ElbowY = (l(1)+l(2)) * sin(deg2rad(t2));
    %W_ElbowZ = (-(l(1)+l(2))) * cos(deg2rad(t1)) * cos(deg2rad(t2));

    %FK WRIST REAL
    W_WristX = -(l(3)+l(4)+l(5))*sin(deg2rad(t(4)))*(cos(deg2rad(t(1)))*cos(deg2rad(0))+sin(deg2rad(t(1)))*sin(deg2rad(t(2)))*sin(deg2rad(0)))-(l(1)+l(2))*cos(deg2rad(t(2)))*sin(deg2rad(t(1)))-(l(3)+l(4)+l(5))*cos(deg2rad(t(2)))*cos(deg2rad(t(4)))*sin(deg2rad(t(1)));
    W_WristY = (l(1)+l(2))*sin(deg2rad(t(2)))+(l(3)+l(4)+l(5))*cos(deg2rad(t(4)))*sin(deg2rad(t(2)))-(l(3)+l(4)+l(5))*cos(deg2rad(t(2)))*sin(deg2rad(0))*sin(deg2rad(t(4)));
    W_WristZ = (l(3)+l(4)+l(5))*sin(deg2rad(t(4)))*(cos(deg2rad(0))*sin(deg2rad(t(1)))-cos(deg2rad(t(1)))*sin(deg2rad(t(2)))*sin(deg2rad(0)))-(l(1)+l(2))*cos(deg2rad(t(1)))*cos(deg2rad(t(2)))-(l(3)+l(4)+l(5))*cos(deg2rad(t(1)))*cos(deg2rad(t(2)))*cos(deg2rad(t(4)));

    %Recalculated Transversal
    R(2) = sqrt(((E_WristX-W_WristX)^2)+((E_WristY-(W_WristY))^2)+((E_WristZ-W_WristZ)^2));
    R(3) = (l(3)+l(4)+l(5))*sin(deg2rad(t(4)));
    if(P(2) < 0)
    t(3) = (rad2deg(asin((R(2)/2)/R(3))))*2;
    else
    t(3) = 0-((rad2deg(asin((R(2)/2)/R(3))))*2);
    end
    t(5) = 0;%-tr1;
    t(6) = 0;%0-(tr3+tr4);
    t(7) = 0;%-tr2;
end

    %for i = 1:7
        %[~, t(i)] = vrep.simxGetJointPosition(clientID, objectHandle(i), vrep.simx_opmode_blocking);
        %disp(rad2deg(t(i)));
    %end