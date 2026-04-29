% This file provide a demo of using the proposed trajectory modulation method to learn from a single demonstration,
% allowing the trajectory to pass through  all the via-points in real-time.
% 


clc
clear

%% Integrating demos
addpath('func');

load('2Dletters/S.mat');

demo_dt = 0.1; % time interval of data
demoLen=50; % number of datapoints in one demonstration trajectory
alpha = .1; %tuning exp
[Data,demo_dura] = demo_processing(demos,demoLen,demo_dt,alpha); % reformat the data set

   
%% Fuzzy modelling

N_C_R1 =5; % number of fuzzy clusters;
dt = .01; % time interval of the reproduced trajectory. We reproduce a trajectory with 10000 datapoint.
maxIter_fcm = 30;
maxIter_gk = 30;
[C_x_R1,pInvCov_x_R1,p1_u_x_R1] = fuzzymodellingCandGK(Data(1:3,:),demoLen,N_C_R1,maxIter_fcm,maxIter_gk); %modelling Position X
[C_y_R1,pInvCov_y_R1,p1_u_y_R1] = fuzzymodellingCandGK(Data([1 2 4],:),demoLen,N_C_R1,maxIter_fcm,maxIter_gk); %modelling Position Y 


%% Trajectory reproduction

timeinput = dt:dt:demoLen*demo_dt;
len_R1 = size(timeinput,2);
[y_x_R1, U_x]= fuzregre_yout_t1(timeinput,demo_dura,alpha,N_C_R1, C_x_R1, pInvCov_x_R1,p1_u_x_R1, [1 2], [3]); %reproducing Position X
[y_y_R1, U_y]= fuzregre_yout_t1(timeinput,demo_dura,alpha,N_C_R1, C_y_R1, pInvCov_y_R1,p1_u_y_R1, [1 2], [3]); %reproducing Position Y
trajectory_reg_R1 = [y_x_R1;y_y_R1];

%% setting via_point_R1, and the normalized via_time_R1 is 0~1
viaFlag=[1 1 1 1 1 1 1 1 1 1 1]; % determine which via-points are used
viaNum=11;
via_time_R1(1)=0;
via_point_R1(:,1)=[5 8.1];
via_time_R1(2)=0.15;
via_point_R1(:,2)=[0 10]'; 
via_time_R1(3)=0.2;
via_point_R1(:,3)=[-1 7]';  
via_time_R1(4)=0.25;
via_point_R1(:,4)=[-7 9]';  
via_time_R1(5)=0.3;
via_point_R1(:,5)=[-8 6]'; 
via_time_R1(6)=0.35;
via_point_R1(:,6)=[-4 3]'; 
via_time_R1(7)=0.375;
via_point_R1(:,7)=[-6 1]'; 
via_time_R1(8)=0.425;
via_point_R1(:,8)=[-2 -0.5]'; 
via_time_R1(9)=0.45;
via_point_R1(:,9)=[1 2]'; 
via_time_R1(10)=0.5;
via_point_R1(:,10)=[7 2]'; 
via_time_R1(11)=1;
via_point_R1(:,11)=[-6.2 -7]; 

idx1 = (viaFlag == 1);
via_time_R1 = (demoLen*demo_dt*demo_dura)*via_time_R1(1,idx1);
via_point_R1 = via_point_R1(:,idx1); 
tic
%% Trajectory modulation
[y_mx] = fuzregre_modulation_yout(timeinput,demo_dura,alpha,len_R1,N_C_R1, C_x_R1, pInvCov_x_R1, p1_u_x_R1, [1 2], [3], via_time_R1, via_point_R1, [1]); %modulating Position X
[y_my] = fuzregre_modulation_yout(timeinput,demo_dura,alpha,len_R1,N_C_R1, C_y_R1, pInvCov_y_R1, p1_u_y_R1, [1 2], [3], via_time_R1, via_point_R1, [2]); %modulating Position Y
trajectory_mod_R1 = [y_mx;y_my];
toc
% plot
figure
set(gcf,'position',[468,600,800,401])
subplot(121)
title('Repreduced trajectory');
hold on
plot(Data(3,1:demoLen),Data(4,1:demoLen), '--','linewidth',4,'color','k'); 
plot(trajectory_reg_R1(1,:),trajectory_reg_R1(2,:), '-','linewidth',2,'color','g'); 
xlim([-10 11])
ylim([-9 11])

subplot(122)
title('Modulated trajectory');
hold on
plot(trajectory_mod_R1(1,:),trajectory_mod_R1(2,:),'linewidth',2,'color','R');
for i = 1:size(via_point_R1,2)
plot(via_point_R1(1,i),via_point_R1(2,i),'o','markersize',10,'linewidth',1.5, 'color','b')
end
xlim([-10 11])
ylim([-9 11])
