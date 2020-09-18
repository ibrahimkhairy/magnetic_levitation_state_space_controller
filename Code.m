clear
clc
close all

%% The Original System
R = 1;
M = 1;
k = 1;
L = 0.01;
g = 9.8;
yo = 0.5;

syms x1 x2 x3
syms xdot1 xdot2 xdot3
syms v y

%linearization with defining new states

xdot1 = x2;
xdot2 = g - (k/M)*(x3^2/x1);
xdot3 = -(R/L) * x3 + v/L;


a = jacobian([xdot1; xdot2; xdot3],[x1, x2, x3]);
b = jacobian([xdot1; xdot2; xdot3],v);

%initials
x1o = yo;
x2o = 0;
x3o = sqrt((M*g/k)*(x1o));
vo = R*x3o;


A = double(subs(a,[x1, x2, x3, v],[x1o, x2o, x3o, vo]));
B = double(subs(b,[x1, x2, x3, v],[x1o, x2o, x3o, vo]));
% we have three cases: output can equal only one of the states x1, x2 or x3
% depending on the sensor we would use as we are using only one sensor.
% as we will see, when assumed y = x3 system became not observalble
% when assumed y = x1 system overshoot does not meet the specification
C1 = [1 0 0];
C2 = [0 1 0];
C3 = [0 0 1];
C_test = [C1;C2;C3];
D = 0; % in all three cases

%% check original system Stability
poles = eig(A); % it has a RHP pole so the original system is not stable

%% check original system Controlability
CT = rank([B A*B A^2*B]); %CT = 3 so system is controllable

%% check original system Observability
% we have three cases for C
for i = 1:3
    OB = rank([C_test(i,:); C_test(i,:)*A; C_test(i,:)*A^2]); %OB = 3 so system is observable
    if OB < 3
        C_test(i,:) = [];
    end
end
% Cases are reduced to only two cases C1 and C2 concatinated in C_test

%% Feedback of Estimated states

%the test input
t = 0:0.01:0.8;
u = zeros(size(t));

% the ball is initially at 0.05m also all states initialled at 0.05
xo = [0.05 0.05 0.05];
eo = [-0.1 -0.1 -0.1]; % assumed all intinal errors are -0.1

test = 0; %indicator for selecting the appropriate C matrix
ind = 0; %indicator for meeting specifications
for i = 1:2 %C_test loop
    for h=-10:0.5:0 %Real part of poles loop
        for j=-10:0.5:-0.5 %Imaginary part of poles loop
            % state observer poles
            op1 = -100;
            op2 = -101;
            op3 = -102;
            % estimator gain
            L = place(A',C_test(i,:)',[op1 op2 op3])';
            
            % controlled system poles
            cp1 = complex(h,-j);
            cp2 = complex(h, j);
            cp3 = -20;
            % feedback gain
            K = place(A,B,[cp1 cp2 cp3]);
            
            %controlled system without observer
            CONT = ss(A-B*K,B,C_test(i,:),D);
            [output_cont,t,x_cont] = lsim(CONT,u,t,xo);
            CONT_prop = lsiminfo(x_cont(:,1),t,0);
            
            %controlled system with the observer
            Atot = [A-B*K B*K; zeros(size(A)) A-L*C_test(i,:)];
            Btot = [B; zeros(size(B))];
            Ctot = [C_test(i,:) zeros(size(C_test(i,:)))];
            Dtot = 0;
            
            TOT_SYS = ss(Atot,Btot,Ctot,Dtot);
            [output_tot,t,x_tot] = lsim(TOT_SYS,u,t,[xo eo]);
            TOT_SYS_prop = lsiminfo(x_tot(:,1),t,0);
            
            if ( max(CONT_prop.SettlingTime,TOT_SYS_prop.SettlingTime) < 0.45 )
                if( max(CONT_prop.Max,TOT_SYS_prop.Max) < 0.055)
                    ind = 1;
                    break;
                end
            end
        end
        if(ind == 1)
            break
        end
    end
    if(ind == 1)
        break
    end
    test = 1;
end

display(A)
display(B)
if test == 1
    C = C_test(2,:);
else
    C = C_test(1,:);
end
display(C)
display(D)
Desired_poles = [cp1 ;cp2];
display(Desired_poles)
Settling_time = TOT_SYS_prop.SettlingTime;
display(Settling_time)
Maximum_deviation = TOT_SYS_prop.Max;
display(Maximum_deviation)

figure
plot(t,x_tot(:,1),'black')
hold on
plot(t,x_cont(:,1),'red')
title('Total System Response Vs State-Feedback System Response')
xlabel('Time (sec)')
ylabel('Ball Deviation From Reference (m)')
legend('with Observer','without observer')
grid on



