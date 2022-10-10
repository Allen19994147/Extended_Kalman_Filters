function SenseDataUC = SensorModelUC(X,LM)
% Scan threw six LM given as map, here each LM is distinct
% Input argument
% X : actual state, [X, Y, Theta]'
% LM : land mark position
% Output argument
% Sensordata : each row [flag, distance, relative angle]
% flag = 1 if corresponding LM detected

LM_X = LM(1,:);
LM_Y = LM(2,:);
LMc = size(LM,2);
% LM_X = [-20 -20 -5  30   30 8];
% LM_Y = [-25  0  25  -25  0  25];
% LMc = length(LM_X);

SenseData = zeros(LMc,3);
Q=[
    0.02    0.00;
    0.00    0.02;
    ];

for id = 1:LMc
    
    % A radius based detection, use physical distance!
    q = sqrt( (LM_X(id)-X(1))^2+(LM_Y(id)-X(2))^2 );
    if  q < 12.0
        
        ran = mvnrnd([q atan2(LM_Y(id)-X(2),LM_X(id)-X(1))-X(3)],Q);
        
        SenseData(id,1) = 1;
        SenseData(id,2) = ran(1);
%         if ran(2) > pi
%             SenseData(id,3) = ran(2) - 2*pi;
%         elseif ran(2) < -pi
%             SenseData(id,3) = ran(2) + 2*pi;
%         else
%             SenseData(id,3) = ran(2);
%         end
        SenseData(id,3) = ThetaRegulator(ran(2)); %%%Confind the theta to  +-pi
    end
end

% Removing correpondance information
SenseData = SenseData(SenseData(:,1)==1, :);
N = size(SenseData,1);
if(N > 0)
    SenseDataUC = SenseData(randperm(N),:);
else
    SenseDataUC = [];
end
end