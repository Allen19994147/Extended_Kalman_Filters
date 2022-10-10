%%%An example of EKF localizing based on work of "Ayush Dewan"
%%%Modified by PooPoo
%%% HW4 EKF localization with known correspondance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Do not move code below %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear
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
%%%%-----------------------
% appendX = [40 50 -55 -12 3 -34 -50];
% appendY = [45 -40 -45 34 -15 -20 40];
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
         
X_all = X;       
X_predicted_all = X_predicted;  %%% Use for Recording all prediction of X
counter = 0;    %%count #interation
% while(1)
for kkk = 1:200
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
    %%% Don't use them in use them
    X = VehicleModel(v,w,previous_pose);    
    previous_pose = X;
    SenseData = SensorModel(X,LM);      % or SensorModelUC(X,LM);
   
    
    %%% -----------MY append-----------------------------------

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
    prob = 0;
    observed_num = sum(SenseData(:,1));
    fprintf('Epoch:%d , Saw %d\n',counter,observed_num);
    if(observed_num < 2)
        observed_num = 1;
    end
    
    for i = 1:LM_c
        if(SenseData(i,1)==1 && SenseData(i,2)<12)  %If it's effective measurement
            q = sqrt((LM_X(i)-X_no_noise(1))^2 + ((LM_Y(i)-X_no_noise(2))^2));
%             if(q>15)        
%                 continue;
%             end
            thi = atan2((LM_Y(i)-X_no_noise(2)),(LM_X(i)-X_no_noise(1)))-X_no_noise(3);
            Zest = [q;ThetaRegulator(thi)];
            H = [-(LM_X(i)-X_no_noise(1))/q       -(LM_Y(i)-X_no_noise(2))/q        0;
                (LM_Y(i)-X_no_noise(2))/(q^2)     -(LM_X(i)-X_no_noise(1))/(q^2)    -1;
                ];
            S = H*cov*H'+Q; 
            K = cov*H'*inv(S);
            
            diff = SpecialModify(SenseData,Zest,i);   %%%Specially modified
            modi = K*diff;
            if(abs(modi(3))>0.5||(abs(modi(1))+abs(modi(2))>2||Zest(1)>12))
            %if(modi(3)>0.2)
                modi
                SenseData(i,2:3)'
                Zest
            end
            
            %%% Probability calculation
            pbb = 1;
            pbb_sum = (det(2*pi.*S)^-0.5)*exp(-0.5.*(diff)'*inv(S)*diff);
            %pdf = mvnpdf(diff,[0;0],S)            
            %%%---------------------------------------------
            %X_correction = X_correction + (K*(SenseData(i,2:3)'-Zest));
            X_correction = X_correction + modi;
            cov =  (eye(size(K*H,1)) - K*H)*cov;
        
        end
        
    end
    [eigenvector,eigenvalue] = eig(cov);
    X_correction;
    X_predicted = X_no_noise + X_correction./observed_num;     %%% prediction X is the final correction of X
    X_predicted(3) =  ThetaRegulator(X_predicted(3));
    %X_predicted_all = [X_predicted_all X_predicted];%%% Record all prediction of X
    
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