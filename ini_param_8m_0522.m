clc
clear;
%% cal
deg2rad(45)
%% 基础参数
H = 1000;
attitude_0 = [0; -90;  -0];
Ts = 0.05;%电机时间常数
Ts_sur = 0.25;%0.25-0.5
%% 机架参数 M=r×F
% --- 1. 机架几何参数 ---
X_F = 1.7; X_B = 1.7; 
Y_1 = 0.5; Y_2 = 1.5;
Z_1 = 1.8; Z_2 = 1.8;
StructAircraft.Motor_Locs = [
         X_F,  Y_2,  -Z_1, -1;  % M1: 右前外正转（+1）：顺时针旋转
         X_F,  Y_1,  -Z_1,  1;  % M2: 前内
         X_F, -Y_1,  -Z_1, -1;  % M3: 前内
         X_F, -Y_2,  -Z_1,  1;  % M4: 前外
        -X_B,  Y_2,   Z_2,  1;  % M5: 右后外
        -X_B,  Y_1,   Z_2, -1;  % M6: 后内
        -X_B, -Y_1,   Z_2,  1;  % M7: 后内
        -X_B, -Y_2,   Z_2, -1;  % M8: 后外
                             ];
Y_2 = 1.8;
StructAircraft.Surface_Locs = [
    X_F,  Y_2 , -Z_1; %↗
    X_F,  Y_2 , -Z_1; %↗
    X_F, -Y_2 , -Z_1; %↖
    X_F, -Y_2 , -Z_1; %↖
   -X_B,  Y_2 ,  Z_2;
   -X_B,  Y_2 ,  Z_2;
   -X_B, -Y_2 ,  Z_2;
   -X_B, -Y_2 ,  Z_2;
    ];
StructAircraft.m = 224;
StructAircraft.TiltXd = 90;%电机安装角
StructAircraft.Km = 0.0435;%电机拉力/电机力矩 N/nm

% [A_motor, B_motor, A_surface, B_surface] = generate_full_mixer_px4_V2(StructAircraft, 25);
[A_norm, A, B, B_motor, B_surface] = generate_full_mixer_px4_V3(StructAircraft, 25);
% StructAircraft.A_motor = A_motor;
StructAircraft.B_motor = B_motor;
% StructAircraft.A_surface = A_surface;
StructAircraft.B_surface = B_surface;
% 

% StructAircraft.inertia = diag([188.14;459.3;592.4]);
StructAircraft.inertia = [
    381.811007, 0, 206.727325;
    0, 459.274905, 0;
    206.727325, 0, 398.710635; ];
%机翼参数
StructAircraft.c_shrink = 0.15;     % 滑流收缩系数
StructAircraft.alpha_swirl = 0.3;   % 旋流强度修正系数
StructAircraft.c_wing = 0.75;          % 机翼弦长
StructAircraft.D = 0.9;             % 桨直径 (m)
StructAircraft.rho = 1.225; 
StructAircraft.S_wing = 1;          % 机翼总面积 (m^2)
StructAircraft.x_dist = 0.05; %桨盘到机翼前缘的距离 (m)
StructAircraft.CL_wing_base = 0.367528201914; % 当前基准迎角下的翼面基础升力系数 (舵面中立时)
StructAircraft.dCL_dDelta = -0.0092;

% run('AeroData.m')
load aeroData.mat
load motorData.mat
%% 气动参数设置
S = 6.0; % m2 (Total Wing planform area)                                                                    
cbar = 0.75; % m; Mean Aerodynamic Chord (MAC)                                                                   
b = 4.0; % m (Wing Span)


%% 悬停油门反算
poly_a = 50;  % 二次项系数 (主要贡献)
poly_b = 3.087;  % 一次项系数
delta = poly_b^2 + 4 * poly_a * StructAircraft.m/8;
hover_throttle = (-poly_b + sqrt(delta)) / (2 * poly_a);
%%
dt = 0.004;
ramp_t = 3;%推力斜坡时间
n_0 = 0.05;%怠速油门
n_1 = hover_throttle*1.05;
k_ramp = (n_1 - n_0)/ramp_t;
%%
%%% delta_attitude = [0; 50;  0];
%%%位置环限制
UAV.pos_x.kp = 2;
UAV.pos_x.ki = 0.05;
UAV.pos_x.kd = 1.20;

UAV.pos_y.kp = 2.0;
UAV.pos_y.ki = 0.05;
UAV.pos_y.kd = 1.20;

UAV.pos_z.kp = 2;
UAV.pos_z.ki = 0.05;
UAV.pos_z.kd = 1;

UAV.pos_x.v_lim = 50;
UAV.pos_y.v_lim = 50;
UAV.pos_z.v_lim = 50;
%%% 速度环=====================

UAV.v2acc.kp_x = 5;
UAV.v2acc.ki_x = 0.50;
UAV.v2acc.kd_x = 2.50;%加微分是可以加快并抑制超调的，从小开始加

UAV.v2acc.kp_y = 4;
UAV.v2acc.ki_y = 0.5;
UAV.v2acc.kd_y = 2.5;

UAV.v2acc.kp_z = 4;
UAV.v2acc.ki_z = 0.6;
UAV.v2acc.kd_z = 0.05;

UAV.pos_x.a_lim = 5;
UAV.pos_y.a_lim = 5;
UAV.pos_z.a_lim_up = 2;
UAV.pos_z.a_lim_down = -3;
%%%高度环 P PID
%%%角度环 P
UAV.P_att = [1; 1; 1];
%%%角速度环 PID
% K_ff = 5.0;
roll_rate_K = 1;
UAV.roll_rate_KP = 2;
UAV.roll_rate_KI = 0.01;
UAV.roll_rate_KD = 0.005;

pitch_rate_K = 1;
UAV.pitch_rate_KP = 3;%min:0.01
UAV.pitch_rate_KI = 0.01;
UAV.pitch_rate_KD = 0.005;
pitch_rate_lim = 1;

yaw_rate_K = 1;
UAV.yaw_rate_KP = 4.5;
UAV.yaw_rate_KI = 0.05;
UAV.yaw_rate_KD = 0.5;
%%
% StructAeroDyn = DefineAerodynamicPara(); 
% StructAircraft = DefineAircraftPara();
% ================================
% UAV.dt = 0.004;
% UAV_PARAM = Simulink.Parameter;
% UAV_PARAM.Value = UAV;
% % UAV_PARAM.StorageClass = 'SimulinkGlobal';
% 
% Simulink.Bus.createObject(UAV)
% fprintf("已更新")
%%
% MC_PITCHRATE_K 最小：0.0100最大：5.0000默认：1.0000
% MC_PITCHRATE_P  最小：0.010最大：0.600默认：0.150
%% 200kg
t = datetime('now');
datestr(t, 'yyyy-mm-dd HH:MM:SS')
