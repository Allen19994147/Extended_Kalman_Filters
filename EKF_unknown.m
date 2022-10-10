%%%An example of EKF localizing based on work of "Ayush Dewan"
%%%Modified by PooPoo
%%% HW4 EKF localization with unknown correspondance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Do not move code below %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear all
clear figure
v = 0;
w = 0;
del_t = 1;
%%%Stochastic uncertainty gain
c1=0.05;
c2=0.05;
c3=0.05;
c4=0.05;
Q=[
    0.02    0.00;
    0.00    0.02;
    ];

%%%Land marks position
LM_X = [-20 -20 -5  30   30 8];
LM_Y = [-25  0  25  -25  0  25];
%%%%------Target extension below-----------------
% appendX = [40 50 -55 -12 3 -34 -50 7 -10 -19  20 15];
% appendY = [45 -40 -45 34 -15 -20 40  -6 10 -10  25 30];
% LM_X = [LM_X appendX];
% LM_Y = [LM_Y appendY];
%%%%-----------------------
LM_c = length(LM_X);
LM = [LM_X;LM_Y];
axis ([-70 70 -70 70]);
%axis ([-35 35 -35 35]);
hold on;
for i = 1:length(LM_X)
    scatter(LM_X(i), LM_Y(i), 'filled', 'd','black');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Do not move code above %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Cart initial position
x_p = 2.5;
y_p = -20;
phi_p = 0;          % Point Y initially
previous_pose = [x_p;y_p;phi_p];
X = previous_pose;
X_predicted = X;    % You can only access "actual state" here initially 

%%% -----------MY append-----------------------------------
cov = [0 0 0;0 0 0;0 0 0];        %%%initial covariance
%cov = eye(3);           
X_all = X;       
counter = 0;    %%count #interation

% while(1)
for kkk = 1:600
    counter = counter +1;
    % If navigation
    % [v,w] = navigation();

    % Move in Circle without hesitate
    v = 1.0;
    w = 0.04;
%     v = 1.0*abs(cos(2*pi*kkk/123));
%     w = 0.04*abs(sin(2*pi*kkk/123));
    % Physical actual output
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% Do not move code below %%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Both X and previous_X are for real cart states
    %%% Don't use them in this task
    X = VehicleModel(v,w,previous_pose);    
    previous_pose = X;
    SenseData = SensorModelUC(X,LM);
    
    
    %X_all = [X_all X];      %%Record all the actual states of cart
    %%%Use predicted_X to estimate pure dynaimics pose
    X_no_noise = VehicleMotionEstimation(v,w,X_predicted);  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% Do not move code above %%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X_no_noise(3) = ThetaRegulator(X_no_noise(3));
    %%% Do state estimation (EKF for this HW)
    %%% X_predicted = Estimate(v,w,X_previous_predicted,SenseData);
    theta = X_predicted(3);
    
    radius = abs(v/w);  %% for following calculation
    G = [1 0 -radius*cos(theta)+radius*cos(theta+w*del_t);
        0 1 -radius*sin(theta)+radius*sin(theta+w*del_t);
        0 0 1];
    V = [-sin(theta)/w+sin(theta+w*del_t)/w radius/w*(sin(theta)-sin(theta+w*del_t))+radius*del_t*cos(theta+w*del_t);
        cos(theta)/w-cos(theta+w*del_t)/w -radius/w*(cos(theta)-cos(theta+w*del_t))+radius*del_t*sin(theta+w*del_t);
        0 del_t];
    M = [(c1*abs(v)+c2*abs(w))^2     0;
          0                           (c3*abs(v)+c4*abs(w))^2];
    cov = G*cov*G' + V*M*V';
    
    %%% X_correction is for updating according to each observed obj 
    X_correction = [0;0;0];
    observed_num = size(SenseData,1);
    fprintf('Epoch:%d , Saw %d\n',counter,observed_num);
    pbb_set = [];   %%% Collection declaration, must be here
    H_set = [];
    S_set = [];
    Zest_set = [];
    for j = 1:observed_num
        pbb_set = [];   %%% Reset em all fot each sense data
        H_set = [];
        S_set = [];
        Zest_set = [];
        for i = 1:LM_c
            if(SenseData(j,2)<12)  %If it's effective measurement
                q = sqrt((LM_X(i)-X_no_noise(1))^2 + ((LM_Y(i)-X_no_noise(2))^2));
%                 if(q > 15)       %%%For landmarks far away, skip em
%                    continue;
%                 end
                thi = atan2((LM_Y(i)-X_no_noise(2)),(LM_X(i)-X_no_noise(1)))-X_no_noise(3);
                Zest = [q;ThetaRegulator(thi)];
                H = [-(LM_X(i)-X_no_noise(1))/q       -(LM_Y(i)-X_no_noise(2))/q        0;
                    (LM_Y(i)-X_no_noise(2))/(q^2)     -(LM_X(i)-X_no_noise(1))/(q^2)    -1;
                    ];
                S = H*cov*H'+Q;  
                diff = SpecialModify(SenseData,Zest,j);   %%%Specially modified
                %%% Probability calculation
                
                pbb = (det(2*pi.*S)^-0.5)*exp(-0.5.*(diff)'*inv(S)*diff);
                %pbb = mvnpdf(diff,[0;0],S)
                Zest_set = [Zest_set Zest];
                S_set = [S_set S];
                H_set = [H_set H];
                pbb_set = [pbb_set pbb];
            end
         end %%%End for looping through each landmarks
         
         %%%If valid measurement and there's sth for update
         if(SenseData(j,2)<12 && sum(size(pbb_set))>0)
             [dummy index] = max(pbb_set);
%              if(dummy < 0.01)
%                  continue;      %%% if the probability is small, skip it
%              end
             pbb_set = pbb_set./sum(pbb_set);
             [dummy index] = max(pbb_set);
             
              H_set(:,3*index-2:3*index);
              K = cov*H_set(:,3*index-2:3*index)'*inv(S_set(:,2*index-1:2*index));
              diff = SpecialModify(SenseData,Zest_set(:,index),j);   %%%Specially modified
              modi = K*diff;
              if(abs(modi(3))>0.5||(abs(modi(1))+abs(modi(2))>2||Zest(1)>12))
                 fprintf('~~GO wild~~~');
                 modi
                 SenseData(j,2:3)'
                 Zest
              end
              X_correction = X_correction + modi;
              cov =  (eye(size(K*H_set(:,3*index-2:3*index),1)) - K*H_set(:,3*index-2:3*index))*cov;
              
         end
    end      %%%End for looping through each observed measurements
    [eigenvector,eigenvalue] = eig(cov);
    
    if(observed_num==0)     %%%If nothing observed
         X_predicted = X_no_noise;
    else
        X_predicted = X_no_noise + X_correction./observed_num;     %%% prediction X is the final correction of X
        X_predicted(3) =  ThetaRegulator(X_predicted(3));
    end
    
     %%% Visualize your result
    if(sum(abs(X_correction)) > 0)
        color = 'green';
    else
        color = 'blue';
    end
    scatter(X(1), X(2), 'filled','red');
    scatter(X_predicted(1), X_predicted(2), [],color);
    
    %%% eclipse
    X0= X_predicted(1);
    Y0= X_predicted(2);
    theta_grid = linspace(0,2*pi);
    x_ellipse = eigenvector(1,1)*sqrt(eigenvalue(1,1)*5.991)*cos(theta_grid) + eigenvector(1,2)*sqrt(eigenvalue(2,2)*5.991)*sin(theta_grid);
    y_ellispe = eigenvector(2,1)*sqrt(eigenvalue(1,1)*5.991)*cos(theta_grid) + eigenvector(2,2)*sqrt(eigenvalue(2,2)*5.991)*sin(theta_grid);
    % Draw the error ellipse
    plot(x_ellipse(1,:) + X0,y_ellispe(1,:) + Y0,'red')
    
    pause(0.01);
    
end