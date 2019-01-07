% Cheng Huimin
% Nov 2018
clear;
close all;
clc;

if ~ exist('bag', 'var')
    bag = rosbag('/home/dhl/git/catkin_ws/src/ekf_sensor_fusion/ssf_updates/matlab/ddrone.bag');
end

% rosgenmsg('/home/dhl/git/catkin_ws/src/ekf_sensor_fusion/')
state_out = select(bag,'Topic','/ekf_fusion/state_out'); % /ekf_fusion/state_out
state_outStruct = readMessages(state_out);

N = length(state_outStruct);

% states
p = zeros(N,3);
v = zeros(N,3);
q = zeros(N,4);
b_w = zeros(N,3);
b_a = zeros(N,3);
lambda = zeros(N,1);
q_wv = zeros(N,4);
q_ci = zeros(N,4);
p_ci = zeros(N,3);

P_p = zeros(N,3);
P_v = zeros(N,3);
P_q = zeros(N,3);
P_bw = zeros(N,3);
P_ba = zeros(N,3);
P_l = zeros(N,1);
P_qci = zeros(N,3);
P_pci = zeros(N,3);

% measurements
a_m = zeros(N,3);
w_m = zeros(N,3);
m_m = zeros(N,3);
q_m = zeros(N,4);

for i = 1:N
    
    % states
    p(i,1) = state_outStruct{i}.Data(1);
    p(i,2) = state_outStruct{i}.Data(2);
    p(i,3) = state_outStruct{i}.Data(3);
    
    v(i,1) = state_outStruct{i}.Data(4);
    v(i,2) = state_outStruct{i}.Data(5);
    v(i,3) = state_outStruct{i}.Data(6);
    
    q(i,1) = state_outStruct{i}.Data(7); % w
    q(i,2) = state_outStruct{i}.Data(8);
    q(i,3) = state_outStruct{i}.Data(9);
    q(i,4) = state_outStruct{i}.Data(10);
    
    b_w(i,1) = state_outStruct{i}.Data(11);
    b_w(i,2) = state_outStruct{i}.Data(12);
    b_w(i,3) = state_outStruct{i}.Data(13);
    
    b_a(i,1) = state_outStruct{i}.Data(14);
    b_a(i,2) = state_outStruct{i}.Data(15);
    b_a(i,3) = state_outStruct{i}.Data(16);
    
    lambda(i) = state_outStruct{i}.Data(17);
    
    q_wv(i,1) = state_outStruct{i}.Data(18);
    q_wv(i,2) = state_outStruct{i}.Data(19);
    q_wv(i,3) = state_outStruct{i}.Data(20);
    q_wv(i,4) = state_outStruct{i}.Data(21);

    q_ci(i,1) = state_outStruct{i}.Data(22);
    q_ci(i,2) = state_outStruct{i}.Data(23);
    q_ci(i,3) = state_outStruct{i}.Data(24);
    q_ci(i,4) = state_outStruct{i}.Data(25);
    
    p_ci(i,1) = state_outStruct{i}.Data(26);
    p_ci(i,2) = state_outStruct{i}.Data(27);
    p_ci(i,3) = state_outStruct{i}.Data(28);
    
    % variance
    P_p(i,1) = state_outStruct{i}.Data(29);
    P_p(i,2) = state_outStruct{i}.Data(30);
    P_p(i,3) = state_outStruct{i}.Data(31);
    
    P_v(i,1) = state_outStruct{i}.Data(32);
    P_v(i,2) = state_outStruct{i}.Data(33);
    P_v(i,3) = state_outStruct{i}.Data(34);
    
    P_q(i,1) = state_outStruct{i}.Data(35);
    P_q(i,2) = state_outStruct{i}.Data(36);
    P_q(i,3) = state_outStruct{i}.Data(37);
    
    P_bw(i,1) = state_outStruct{i}.Data(38);
    P_bw(i,2) = state_outStruct{i}.Data(39);
    P_bw(i,3) = state_outStruct{i}.Data(40);
    
    P_ba(i,1) = state_outStruct{i}.Data(41);
    P_ba(i,2) = state_outStruct{i}.Data(42);
    P_ba(i,3) = state_outStruct{i}.Data(43);
    
    P_l(i) = state_outStruct{i}.Data(44);
    
    P_qci(i,1) = state_outStruct{i}.Data(48);
    P_qci(i,2) = state_outStruct{i}.Data(49);
    P_qci(i,3) = state_outStruct{i}.Data(50);
    
    P_pci(i,1) = state_outStruct{i}.Data(51);
    P_pci(i,2) = state_outStruct{i}.Data(52);
    P_pci(i,3) = state_outStruct{i}.Data(53);
    
    % measurements
    a_m(i,1) = state_outStruct{i}.Data(54);
    a_m(i,2) = state_outStruct{i}.Data(55);
    a_m(i,3) = state_outStruct{i}.Data(56);
    
    w_m(i,1) = state_outStruct{i}.Data(57);
    w_m(i,2) = state_outStruct{i}.Data(58);
    w_m(i,3) = state_outStruct{i}.Data(59);
    
    m_m(i,1) = state_outStruct{i}.Data(60);
    m_m(i,2) = state_outStruct{i}.Data(61);
    m_m(i,3) = state_outStruct{i}.Data(62);
    
    q_m(i,1) = state_outStruct{i}.Data(63);
    q_m(i,2) = state_outStruct{i}.Data(64);
    q_m(i,3) = state_outStruct{i}.Data(65);
    q_m(i,4) = state_outStruct{i}.Data(66); 
end

%% plot position
f1 = figure('Name','Positions and Velocities');

subplot(5,1,1)
hold on
grid on

ts_p = timeseries(p(:,1));
plot(ts_p);
ts_p = timeseries(p(:,2));
plot(ts_p);

title('State: Position XY');
legend('p_x','p_y');

subplot(5,1,2)
hold on
grid on

ts_p = timeseries(p(:,3));
plot(ts_p);

title('State: Position Z');
legend('p_z');

%% plot position variance
subplot(5,1,3)
hold on
grid on

ts_P_p = timeseries(P_p(:,1));
plot(ts_P_p);
ts_P_p = timeseries(P_p(:,2));
plot(ts_P_p);
ts_P_p = timeseries(P_p(:,3));
plot(ts_P_p);
title('Variance: Position');
legend('Pp_x','Pp_y','Pp_z');

%% plot velocity
subplot(5,1,4)
hold on
grid on

ts_v = timeseries(v(:,1));
plot(ts_v);
ts_v = timeseries(v(:,2));
plot(ts_v);
ts_v = timeseries(v(:,3));
plot(ts_v);
title('State: Velocity');
legend('v_x','v_y','v_z');


%% plot velocity variance
subplot(5,1,5)
hold on
grid on

ts_P_v = timeseries(P_v(:,1));
plot(ts_P_v);
ts_P_v = timeseries(P_v(:,2));
plot(ts_P_v);
ts_P_v = timeseries(P_v(:,3));
plot(ts_P_v);
title('Variance: Velocity');
legend('Pv_x','Pv_y','Pv_z');

%% plot quaternion
f2 = figure('Name', 'Attitude');

subplot(3,1,1)
hold on
grid on

ts_q = timeseries(q(:,1));
plot(ts_q);
ts_q = timeseries(q(:,2));
plot(ts_q);
ts_q = timeseries(q(:,3));
plot(ts_q);
ts_q = timeseries(q(:,4));
plot(ts_q);
title('State: Attitude');
legend('q_w','q_x','q_y','q_z');

%% plot q measurement

subplot(3,1,2)
hold on
grid on

ts_qm = timeseries(q_m(:,1));
plot(ts_qm);
ts_qm = timeseries(q_m(:,2));
plot(ts_qm);
ts_qm = timeseries(q_m(:,3));
plot(ts_qm);
ts_qm = timeseries(q_m(:,4));
plot(ts_qm);
title('Measurement: q');
legend('q_w','q_x','q_y','q_z');

%% plot quaternion variance
subplot(3,1,3)
hold on
grid on

ts_P_q = timeseries(P_q(:,1));
plot(ts_P_q);
ts_P_q = timeseries(P_q(:,2));
plot(ts_P_q);
ts_P_q = timeseries(P_q(:,3));
plot(ts_P_q);
title('Variance: Attitude');
legend('Pq_x','Pq_y','Pq_z');

%% plot bias w
f3 = figure('Name', 'Biases');

subplot(5,1,1)
hold on
grid on

ts_bw = timeseries(b_w(:,1));
plot(ts_bw);
ts_bw = timeseries(b_w(:,2));
plot(ts_bw);
ts_bw = timeseries(b_w(:,3));
plot(ts_bw);
title('State: Gyroscope Bias');
legend('bw_x','bw_y','bw_z');

%% plot P of bias w

subplot(5,1,2)
hold on
grid on

ts_Pbw = timeseries(P_bw(:,1));
plot(ts_Pbw);
ts_Pbw = timeseries(P_bw(:,2));
plot(ts_Pbw);
ts_Pbw = timeseries(P_bw(:,3));
plot(ts_Pbw);
title('Variance: Gyroscope Bias');
legend('Pbw_x','Pbw_y','Pbw_z');


%% plot bias a

subplot(5,1,3)
hold on
grid on

ts_ba = timeseries(b_a(:,1));
plot(ts_ba);
ts_ba = timeseries(b_a(:,2));
plot(ts_ba);
ts_ba = timeseries(b_a(:,3));
plot(ts_ba);
title('State: Accelerometer Bias');
legend('ba_x','ba_y','ba_z');


%% plot P of bias a

subplot(5,1,4)
hold on
grid on

ts_Pba = timeseries(P_ba(:,1));
plot(ts_Pba);
ts_Pba = timeseries(P_ba(:,2));
plot(ts_Pba);
ts_Pba = timeseries(P_ba(:,3));
plot(ts_Pba);
title('Variance: Accelerometer Bias');
legend('Pba_x','Pba_y','Pba_z');

%% plot scale lambda

subplot(5,1,5)
hold on
grid on

ts_lambda = timeseries(lambda(:));
plot(ts_lambda);
title('State: Scale Lambda');
legend('L');


%% plot magnetic vector

f4 = figure('Name', 'Others');

subplot(3,1,1)
hold on
grid on
ts_m_m = timeseries(m_m(:,1));
plot(ts_m_m);
ts_m_m = timeseries(m_m(:,2));
plot(ts_m_m);
ts_m_m = timeseries(m_m(:,3));
plot(ts_m_m);
title('Measurement: Magnetic Field');
legend('m_x','m_y','m_z');


%% plot calibration p_ci

subplot(3,1,2)
hold on
grid on

ts_p_ci = timeseries(p_ci(:,1));
plot(ts_p_ci);
ts_p_ci = timeseries(p_ci(:,2));
plot(ts_p_ci);
ts_p_ci = timeseries(p_ci(:,3));
plot(ts_p_ci);
title('State: Calibration p_{ci}');
legend('pci_x','pci_y','pci_z');


%% plot calibration q_ci

subplot(3,1,3)
hold on
grid on

ts_q_ci = timeseries(q_ci(:,1));
plot(ts_q_ci);
ts_q_ci = timeseries(q_ci(:,2));
plot(ts_q_ci);
ts_q_ci = timeseries(q_ci(:,3));
plot(ts_q_ci);
ts_q_ci = timeseries(q_ci(:,4));
plot(ts_q_ci);
title('State: Calibration q_{ci}');
legend('qci_w','qci_x','qci_y','qci_z');
