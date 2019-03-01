
clear;
clc;
close all;

tic;
for time = 1:1000
random.state = 10;

ActualPos = rand(1,6);
ActualVel = rand(1,6);
ActualAcc = zeros(1,6);
Tor = rand(6,1);
% function Pos = Robot_model(Pos_input, Vel_input, Tor_input)

Ts = 0.001;
% Pos_joint1 = Pos_input(1);
% Pos_joint2 = Pos_input(2);
% Pos_joint3 = Pos_input(3);
% Pos_joint4 = Pos_input(4);
% Pos_joint5 = Pos_input(5);
% Pos_joint6 = Pos_input(6);
% ActualPos = [Pos_joint1 Pos_joint2 Pos_joint3 Pos_joint4 Pos_joint5 Pos_joint6];
% 
% Vel_joint1 = Vel_input(1);
% Vel_joint2 = Vel_input(2);
% Vel_joint3 = Vel_input(3);
% Vel_joint4 = Vel_input(4);
% Vel_joint5 = Vel_input(5);
% Vel_joint6 = Vel_input(6);
% ActualVel = [Vel_joint1 Vel_joint2 Vel_joint3 Vel_joint4 Vel_joint5 Vel_joint6];
% 
% Tor_joint1 = Tor_input(1);
% Tor_joint2 = Tor_input(2);
% Tor_joint3 = Tor_input(3);
% Tor_joint4 = Tor_input(4);
% Tor_joint5 = Tor_input(5);
% Tor_joint6 = Tor_input(6);
% Tor = [Tor_joint1 Tor_joint2 Tor_joint3 Tor_joint4 Tor_joint5 Tor_joint6]';

% ActualAcc = zeros(1,6);


%% compute the Cdq+Gq
% addition 2018/11/7
iVector = [0,0,0,0,85.637165,0;-32.936597,-12.423933,-4.210268,-6.756928,114.068543,0;-6.714691,8.191622,-0.528463,-3.398779,8.076524,0;-2.855767,0.244458,-2.404464,1.330123,2.098873,0;0.659270,0.119476,0.328334,-0.077258,-0.525846,0;0.998351,-0.210655,-0.005650,0.166542,0.431593,0];
mCenter = [0,56.692249,8.951838,-0.439562,0.282232,-0.008820;0,-0.319467,-10.731276,0.242905,0.608514,-0.099975;0,0,0,0,0,0];
Ia = [22.070241 -2.031883 4.303011 0.451409];
Fs = [42.602849 78.165649 49.025961 5.472500 40.078730 12.640710];
Fv = [183.130171 175.360463 76.813632 21.388741 41.831148 17.264665];
% addition 2018/11/7

    DH = [0, 0, 0, 0; 0, 0.19, pi/2, pi/2; 0, 0.65, 0, 0; 0.73, 0.192, pi/2, 0; 0, 0, pi/2, 0; 0, 0, -pi/2, 0];
    iVector = [0,0,0,0,0.844903,0;-0.324345,-0.075914,-0.051391,-0.042523,0.740601,0;-0.091039,0.053809,0.006962,-0.031913,0.049886,0;-0.009236,-0.006692,-0.011727,-0.000768,0.020729,0;-0.004762,0.002377,-0.001658,0.003359,0.000719,0;-0.005854,-0.004165,-0.005599,0.002253,0.001373,0];
    mCenter = [0,0.327776,0.067582,-0.004515,0.003987,0.000929;0,0.021804,-0.089291,0.002001,0.00702,-0.001923;0,0,0,0,0,0];
    Ia = [0.259947,-0.014791,0.040481,0.013916];
    Fs = [0.372305,0.494409,0.409654,0.063411,0.32759,0.197599];
    Fv = [1.610821,1.074123,0.624158,0.264797,0.327863,0.311985];   
    %tor = zeros(6, 1);
    
    tor = zeros(6, 1);
    iMatrix=zeros(3,3,6);
    rot=zeros(3,3,6);
    omega=zeros(3,6);
    domega=zeros(3,6);
    accer=zeros(3,6);
    cforce=zeros(3,6);
    force=zeros(3,6);
    torque=zeros(3,6);

    
    for i = 1 : 6
        iMatrix(:, :, i) = [iVector(i, 1), iVector(i, 2), iVector(i, 3); iVector(i, 2), iVector(i, 6), iVector(i, 4); iVector(i, 3), iVector(i, 4), iVector(i, 5)];
    end
    qval = ActualPos(1, :);
    dqval = ActualVel(1, :);
    ddqval = ActualAcc(1, :);
    theta = DH(:, 4)' + qval;
    pos = [DH(:, 2)'; -(DH(:, 1).*sin(DH(:, 3)))'; (DH(:, 1).*cos(DH(:, 3)))'];
    for i = 1 : 6
       rot(:, :, i) = [cos(theta(i)), cos(DH(i, 3))*sin(theta(i)), sin(DH(i, 3))*sin(theta(i)); -sin(theta(i)), cos(DH(i, 3))*cos(theta(i)), sin(DH(i, 3))*cos(theta(i)); 0, -sin(DH(i, 3)), cos(DH(i, 3))];
    end
    i = 0;
    omega_0 = zeros(3, 1);
    domega_0 = zeros(3, 1);
    accer_0 = [0; 0; 9.80665];
    i = 1;
    omega(:, i) = [0; 0; dqval(i)];
    domega(:, i) = [0; 0; ddqval(i)];
    accer(:, i) = rot(:, :, i) * accer_0;
    cforce(:, i) = cross(domega(:, i), mCenter(:, i)) + cross(omega(:, i), cross(omega(:, i), mCenter(:, i)));
    for i = 2 : 6
        omega(:, i) = rot(:, :, i) * omega(:, i - 1) + [0; 0; dqval(i)];
        domega(:, i) = rot(:, :, i) * domega(:, i - 1) + cross(rot(:, :, i)*omega(:, i - 1), [0; 0; dqval(i)]) + [0; 0; ddqval(i)];
        accer(:, i) = rot(:, :, i) * (accer(:, i - 1) + (cross(domega(:, i - 1), pos(:, i))) + (cross(omega(:, i - 1), cross(omega(:, i - 1), pos(:, i)))));
        cforce(:, i) = cross(domega(:, i), mCenter(:, i)) + cross(omega(:, i), cross(omega(:, i), mCenter(:, i)));
    end
    i = 6;
    force(:, i) = cforce(:, i);
    torque(:, i) = iMatrix(:, :, i) * domega(:, i) + cross(omega(:, i), iMatrix(:, :, i)*omega(:, i)) + cross(mCenter(:, i), accer(:, i));
    tor(i) = torque(3, i); 
    for i = 5 : -1 : 1
        force(:, i) = cforce(:, i) + rot(:, :, i+1)' * force(:, i+1);
        torque(:, i) = iMatrix(:, :, i) * domega(:, i) + cross(omega(:, i), iMatrix(:, :, i)*omega(:, i)) + cross(mCenter(:, i), accer(:, i)) + rot(:, :, i+1)' * torque(:, i+1) + cross(pos(:, i+1), rot(:, :, i+1)'*force(:, i+1));
        tor(i) = torque(3, i);
    end
    tor = tor + (Fs.*sign(dqval))' + (Fv.*dqval)';
%     tor = tor + (Fs.*sign(dqval))' + (Fv.*dqval)' + ([0, 0, Ia].*ddqval)';

%     tor = tor';
    Tor_delta = Tor - tor;


     
%% compute the Hq

    Eye_matrix = eye(6);        
    tor = zeros(6, 1);
    iMatrix=zeros(3,3,6);
    rot=zeros(3,3,6);
    omega=zeros(3,6);
    domega=zeros(3,6);
    accer=zeros(3,6);
    cforce=zeros(3,6);
    force=zeros(3,6);
    torque=zeros(3,6);
    H = zeros(6,6);
    
    ActualVel2 = zeros(1,6);
    
    for j = 1:6
        ActualAcc = Eye_matrix(:,j)';
        qval = ActualPos(1, :);
        dqval = ActualVel2(1, :);
        ddqval = ActualAcc(1, :);
        theta = DH(:, 4)' + qval;
        pos = [DH(:, 2)'; -(DH(:, 1).*sin(DH(:, 3)))'; (DH(:, 1).*cos(DH(:, 3)))'];
        for i = 1 : 6
            rot(:, :, i) = [cos(theta(i)), cos(DH(i, 3))*sin(theta(i)), sin(DH(i, 3))*sin(theta(i)); -sin(theta(i)), cos(DH(i, 3))*cos(theta(i)), sin(DH(i, 3))*cos(theta(i)); 0, -sin(DH(i, 3)), cos(DH(i, 3))];
        end
        i = 0;
        omega_0 = zeros(3, 1);
        domega_0 = zeros(3, 1);
        accer_0 = [0; 0; 0];   %  设置加速度为0
        i = 1;
        omega(:, i) = [0; 0; dqval(i)];
        domega(:, i) = [0; 0; ddqval(i)];
        accer(:, i) = rot(:, :, i) * accer_0;
        cforce(:, i) = cross(domega(:, i), mCenter(:, i)) + cross(omega(:, i), cross(omega(:, i), mCenter(:, i)));
        cforce(:, i) = cross(domega(:, i), mCenter(:, i)) + cross(omega(:, i), cross(omega(:, i), mCenter(:, i)));
        for i = 2 : 6      
            omega(:, i) = rot(:, :, i) * omega(:, i - 1) + [0; 0; dqval(i)];
            domega(:, i) = rot(:, :, i) * domega(:, i - 1) + cross(rot(:, :, i)*omega(:, i - 1), [0; 0; dqval(i)]) + [0; 0; ddqval(i)];
            accer(:, i) = rot(:, :, i) * (accer(:, i - 1) + (cross(domega(:, i - 1), pos(:, i))) + (cross(omega(:, i - 1), cross(omega(:, i - 1), pos(:, i)))));
            cforce(:, i) = cross(domega(:, i), mCenter(:, i)) + cross(omega(:, i), cross(omega(:, i), mCenter(:, i)));
        end
        i = 6;
        force(:, i) = cforce(:, i);
        torque(:, i) = iMatrix(:, :, i) * domega(:, i) + cross(omega(:, i), iMatrix(:, :, i)*omega(:, i)) + cross(mCenter(:, i), accer(:, i));
        tor(i) = torque(3, i); 
        for i = 5 : -1 : 1
            force(:, i) = cforce(:, i) + rot(:, :, i+1)' * force(:, i+1);
            torque(:, i) = iMatrix(:, :, i) * domega(:, i) + cross(omega(:, i), iMatrix(:, :, i)*omega(:, i)) + cross(mCenter(:, i), accer(:, i)) + rot(:, :, i+1)' * torque(:, i+1) + cross(pos(:, i+1), rot(:, :, i+1)'*force(:, i+1));
            tor(i) = torque(3, i); 
        end
        
%         H(:,j) = tor + (Fs.*sign(dqval))' + (Fv.*dqval)' + ([0, 0, Ia].*ddqval)';  
        H(:,j) = tor + (Fs.*sign(dqval))' + (Fv.*dqval)';  
        
    end
    

%% compute the ddq   
     Acc = zeros(1,6);
%      Vel = zeros(6,1);
%      Pos = zeros(6,1);
     
     Acc = (inv(H) * Tor_delta)';
     Vel = ActualVel + Acc * Ts;
     Pos = ActualPos + ActualVel * Ts + 0.5 * Acc * (Ts)^2;
end
toc;
     
% end
     
















