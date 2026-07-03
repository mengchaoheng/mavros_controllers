%% main.m
% Simple quadrotor simulation with modular reference trajectories.
% Internal coordinate: NED, z_NED points downward.
%
% State:
%   p : position in NED
%   v : velocity in NED
%   R : body-to-NED rotation matrix
%   Omega : body angular velocity expressed in body frame
%
% Input:
%   T   : total thrust force
%   tau : body-frame moment
%
% Reference interface:
%   ref.p   position
%   ref.v   velocity
%   ref.a   acceleration
%   ref.j   jerk
%   ref.s   snap
%   ref.psi yaw
%   ref.psiDot yaw rate
%   ref.psiDDot yaw acceleration

if exist('UAV_BENCHMARK_BATCH', 'var') && UAV_BENCHMARK_BATCH
    if exist('UAV_BENCHMARK_PAR_OVERRIDE', 'var')
        parOverride__ = UAV_BENCHMARK_PAR_OVERRIDE;
    else
        parOverride__ = struct();
    end
else
    clear; clc; close all;
    parOverride__ = struct();
end

%% ========================================================================
%% 0. Parameters
par.g = 9.81;
par.e3 = [0;0;1];
% Iris Gazebo Classic plant, actuator, and geometry parameters, matching
% iris.m and gazebo_iris_model.m.
par.m = 0.75;
par.J = diag([2.5, 2.1, 4.3])*1e-3;
par.ct = 1.51e-6;                 % motorConstant [N/(rad/s)^2]
par.cq = 2.37e-8;                 % momentConstant coefficient
par.kappa = par.cq/par.ct;        % yaw moment / thrust [m]
par.omegaMax = 2.3726e3;          % max rotor speed [rad/s]
par.CT = 8.5;                     % max thrust per normalized actuator [N]
par.pos = [ ...
     0.13,  0.22, -0.023;
    -0.13, -0.20, -0.023;
     0.13, -0.22, -0.023;
    -0.13,  0.20, -0.023];
par.axis = [0; 0; -1];
par.spin = [1; 1; -1; -1];

par.dt = 1/500;         % Base simulation/hold tick; 100 Hz and 250 Hz divide exactly.
par.Tend = 20.0;
par.integratorName = "ode45";  % "ode45" or "lie_rk4"
par.control.outerPeriod = 0.01;  % 100 Hz MPC solve refresh.
par.control.innerPeriod = 1/250; % 250 Hz non-MPC/INDI/direct-control refresh.
% Shared INDI feedback LPFs. All INDI inner loops use the same cutoff; Tal
% and geometric_indi also share one outer-loop INDI cutoff.
par.indi.innerLoopFilterCutoffHz = 30;
par.indi.outerLoopFilterCutoffHz = 30;

% Available choices:
%   "figure8_horizontal"
%   "figure8_vertical"
%   "helix_flip"
%   "helix_flip_y"
%   "flip_loop_sine"
%   "fast_circle"
par.trajName = "figure8_horizontal";

% Analytic trajectory parameters. All trajectories use the same phase variable:
%   theta(t) = theta0 + integral Omega(t) dt
% and then place standard sin/cos components on selected axes.
% Modes:
%   "fixed"     : omega(t)=omega
%   "linear"    : omega(t) interpolates linearly from omegaRange(1) to (2)
%   "quadratic" : omega(t) interpolates quadratically over the same range
par.traj.omega.mode = "quadratic";

% Horizontal figure-eight:
%   x = Ax*sin(theta), y = Ay*sin(2*theta), z = -Hc
par.traj.figure8_horizontal.Ax = 2.0;
par.traj.figure8_horizontal.Ay = 2.0;
par.traj.figure8_horizontal.Hc = 3.0;
par.traj.figure8_horizontal.theta0 = 0;
par.traj.figure8_horizontal.omega.value = 1;        % fixed angular frequency [rad/s]
par.traj.figure8_horizontal.omega.range = [0.5, 1.5];   % start/end angular frequency [rad/s]

% Vertical figure-eight:
%   x = 0, y = Ay*sin(theta), z = -Hc - Az*sin(2*theta)
par.traj.figure8_vertical.Ay = 2.0;
par.traj.figure8_vertical.Az = 2.0;
par.traj.figure8_vertical.Hc = 3.0;
par.traj.figure8_vertical.theta0 = -pi/4;
par.traj.figure8_vertical.omega.value = 1;        % fixed angular frequency [rad/s]
par.traj.figure8_vertical.omega.range = [0.5, 1.5];   % start/end angular frequency [rad/s]

% X-forward helix/flip loop:
%   x = Vx*t, y = Ay*sin(theta), z = -Hc + Az*cos(theta)
par.traj.helix_flip.Ay = 2.0;
par.traj.helix_flip.Az = 2.0;
par.traj.helix_flip.Hc = 3.0;
par.traj.helix_flip.Vx = 0.30;
par.traj.helix_flip.theta0 = 0;
par.traj.helix_flip.omega.value = 3.8;        % fixed angular frequency [rad/s]
par.traj.helix_flip.omega.range = [2, 4];   % start/end angular frequency [rad/s]

% Y-forward helix/flip loop:
%   x = Ax*sin(theta), y = Vy*t, z = -Hc + Az*cos(theta)
par.traj.helix_flip_y.Ax = 2.0;
par.traj.helix_flip_y.Az = 2.0;
par.traj.helix_flip_y.Hc = 3.0;
par.traj.helix_flip_y.Vy = 0.30;
par.traj.helix_flip_y.theta0 = 0;
par.traj.helix_flip_y.omega.value = 3.8;        % fixed angular frequency [rad/s]
par.traj.helix_flip_y.omega.range = [2, 4];   % start/end angular frequency [rad/s]

% Pure vertical flip loop:
%   x = Vx*t, y = Ay*sin(theta), z = -Hc + Az*cos(theta), with Vx=0
par.traj.flip_loop_sine.Ay = 2.0;
par.traj.flip_loop_sine.Az = 2.0;
par.traj.flip_loop_sine.Hc = 3.0;
par.traj.flip_loop_sine.Vx = 0.0;
par.traj.flip_loop_sine.theta0 = 0;
par.traj.flip_loop_sine.omega.value = 3.8;        % fixed angular frequency [rad/s]
par.traj.flip_loop_sine.omega.range = [2, 4];   % start/end angular frequency [rad/s]

% Horizontal circle/ellipse:
%   x = Ax*cos(theta), y = Ay*sin(theta), z = -Hc
par.traj.fast_circle.Ax = 3.0;
par.traj.fast_circle.Ay = 3.0;
par.traj.fast_circle.Hc = 3.0;
par.traj.fast_circle.theta0 = 0;
par.traj.fast_circle.omega.value = 3.4;        % fixed angular frequency [rad/s]
par.traj.fast_circle.omega.range = [1.5, 3.5];   % start/end angular frequency [rad/s]

% controller
% "geometric", "lee", "johnson", "px4_iris"
% "lu", "sun_nmpc", "sun_nmpc_indi"
% "geometric_indi"
par.controllerName = "johnson";
% Shared acceleration-level gains, using Sun et al. Table I DFBC values as
% the benchmark default. Kp/Kv command linear acceleration; KR/KOmega
% command angular acceleration. Controllers that output force/moment convert
% with m and J in their own namespace below.
par.Kp = diag([10, 10, 10]);
par.Kv = diag([6, 6, 6]);
par.KR = diag([150, 150, 3]);
par.KOmega = diag([20, 20, 8]);

% Controller-specific gain namespaces. The base gains above remain as
% convenient defaults, but controller code should use its own namespace so
% paper implementations can be tuned independently.
% Geometric baseline controller. No author-specific reference namespace.
par.geometric.Kp = par.Kp;
par.geometric.Kv = par.Kv;
par.geometric.KR = par.KR;
par.geometric.KOmega = par.KOmega;

% [1] T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” Mar. 10, 2010, arXiv: arXiv:1003.2005. doi: 10.48550/arXiv.1003.2005.
par.lee.Kp = par.m*par.Kp;
par.lee.Kv = par.m*par.Kv;
par.lee.KR = par.J*par.KR;
par.lee.KOmega = par.J*par.KOmega;

% [2] J. Johnson and R. Beard, “Globally-Attractive Logarithmic Geometric Control of a Quadrotor for Aggressive Trajectory Tracking,” Dec. 01, 2021, arXiv: arXiv:2109.07025. doi: 10.48550/arXiv.2109.07025.
par.johnson.We = diag([22.0, 22.0, 355.0, ...
                             2.3, 2.3, 27.0, ...
                             1e-3, 1e-3, 0.1]);
par.johnson.Wf = diag([0.1, 0.1, 1.0]);
par.johnson.positionGainMode = "pd";  % "pd" aligns gains; "lqr" uses We/Wf.
par.johnson.Kp = par.m*par.Kp;
par.johnson.Kv = par.m*par.Kv;
par.johnson.Ki = zeros(3);
par.johnson.Kr = par.J*par.KR;
par.johnson.Komega = par.J*par.KOmega;

% Unified actuator model and control allocation.
% All non-PX4 controllers produce the physical wrench mu = [T; tau].
% PX4 produces normalized actuator commands and is converted separately.
par.allocation.method = "wls";  % Unified allocation: "wls" or "pinv".
% WLS follows the QCAT/test.m convention: Wv=I, Wu=I, ud=0, gamma large,
% and u0 at the actuator-box midpoint.
par.allocation.gamma = 1e6;
par.allocation.uMin = zeros(4,1);
par.allocation.uMax = par.CT*ones(4,1);

% Sun Eq. (9) aerodynamic model.
% The body frame here is FRD/NED: body +z is opposite the collective thrust.
par.aero.enabled = false;
par.aero.kd = [0.16; 0.18; 0.22];
par.aero.kh = 0.01;

% PX4 Iris controller: position -> attitude -> rate -> allocation.
% gainSource="benchmark_equivalent" maps the shared benchmark gains
% Kp/Kv/KR/KOmega into PX4's cascaded normalized-torque controller:
% 一方面是串级，另一方面是% normalize allocation: D*[T;tau]=D*B*U*u_norm,  B*U = B_px4,
% D*B*U = B_norm，D=f(B*U)的计算参考px4_normalize_B，并且可以分为两部分D_T，D_tau
% 控制器输出都是加速度/角加速度量纲：
%   benchmark: a_fb = (Kp*e_p + Kv*e_v) 
%   PX4:       v_sp = v_ref + K_pos*e_p
%              a_sp = K_vel*(v_sp-v) = K_vel*K_pos*e_p + K_vel*e_v
% 单看力，T=m*a，并且在px4中：
% T_c = a_sp * h/g = a_sp * m/4T_max, h=m*g/4T_max.  
% T_c 是px4的位置控制的输出，然后使用B_norm经过单位化分配后得到u_norm, 
% 事实上 D_T = 1/4T_max, T_sp = m*a_sp
% T_c=T_sp/4T_max是一个力单位化过程。加速度控制部分显式计算了控制器输出的单位化（使用h/g）。
% => K_vel = diag(Kv) , 
%   match: K_pos = diag(Kp)./diag(Kv)
%          K_vel = diag(Kv)

% For attitude/rate feedback:
%   benchmark: OmegaDot_fb = KR*e_R + KOmega*(Omega_ref-Omega)
%   PX4:       Omega_sp = K_att*e_R + Omega_ff
%              tau_sp = K_rate*(Omega_sp-Omega)
% 单看力矩, 在px4中与加速度不同，角加速度控制部分则直接认为已经单位化然后单位化控制分配：
% tau_sp =  B_norm*u_norm 
% 因此 tau_sp = D_tau*J*OmegaDot_fb
%   match:     K_att = diag(KR)./diag(KOmega),
%              K_rate = D_tau*J*diag(KOmega).
par.px4_iris.hoverThrust = 0.216;
par.px4_iris.gainSource = "benchmark_equivalent"; % "benchmark_equivalent" or "manual"
par.px4_iris.posP = [10/6; 10/6; 10/6];
par.px4_iris.velP = [6.0; 6.0; 6.0];
par.px4_iris.velI = zeros(3,1);
par.px4_iris.velD = zeros(3,1);
par.px4_iris.yawWeight = 1.0;
par.px4_iris.attP = [7.5; 7.5; 0.375];
par.px4_iris.rateLimit = inf(3,1);
par.px4_iris.rateP = [0.01599789; 0.01343823; 0.06753230]; % 实际中和不同仿真略有差异。因为有执行器的影响
par.px4_iris.rateI = zeros(3,1);
par.px4_iris.rateD = [0.0; 0.0; 0.0];
par.px4_iris.rateFF = [0.0; 0.0; 0.0];
par.px4_iris.rateIntLimit = inf(3,1);
par.px4_iris.rateIntLimit = inf(3,1);
par.px4_iris.useAccelerationFeedforward = true;
par.px4_iris.useYawRateFeedforward = true;
par.px4_iris.decoupleHorizontalAndVerticalAcceleration = false;

% Shared MPC comparison parameters from Sun et al. Table I. The OCP grid
% interval dt=50 ms is distinct from the 100 Hz receding-horizon refresh
% period used by the benchmark controllers.
par.mpc.N = 20;
par.mpc.dt = 0.05;
par.mpc.horizon = par.mpc.N * par.mpc.dt;
par.mpc.Qp = diag([200, 200, 500]);
par.mpc.Qv = eye(3);
par.mpc.Qq = diag([5, 5, 200]);
par.mpc.QOmega = eye(3);
par.mpc.Qu = 6*eye(4);

% [5] G. Lu, W. Xu, and F. Zhang, “On-Manifold Model Predictive Control for Trajectory Tracking on Robotic Systems,” IEEE Transactions on Industrial Electronics, vol. 70, no. 9, pp. 9192–9202, Sep. 2023, doi: 10.1109/TIE.2022.3212397.
% Lu et al. on-manifold MPC, quadrotor experiment, Eq. (6)-(16).
% State error: [p-pd; v-vd; Log(Rd'R)], input: [aT-aTd; Omega-OmegaD].
par.lu.N = par.mpc.N;
par.lu.dt = par.mpc.dt;
par.lu.horizon = par.lu.N * par.lu.dt;
par.lu.solvePeriod = par.control.outerPeriod;  % MPC refresh period Tc = 100 Hz.
par.lu.maxQPIt = 120;
par.lu.qpTol = 1e-7;
par.lu.omegaMax = par.px4_iris.rateLimit;  % Always align Lu u=[aT;Omega] rate bound with PX4.
% Lu's MPC input is [thrust acceleration; body rate]. The benchmark plant
% accepts force/moment, so a Lu-local PX4-style body-rate PID converts
% Omega_cmd to physical torque, then the normal benchmark allocation maps
% [m*aT_cmd; tau] to actuator thrusts.
par.lu.ratePeriod = par.control.innerPeriod; % 250 Hz body-rate adapter.
par.lu.rateP = par.J*par.KOmega;
par.lu.rateI = zeros(3);
par.lu.rateD = zeros(3);
par.lu.rateFF = zeros(3);
par.lu.rateIntLimit = inf(3,1);

% Lu MPC objective, mapped from the shared MPC weights above. Lu attitude
% error is Log(Rd'R), while Sun Eq. (12) uses q_e,v ~= 0.5*Log, so the
% shared quaternion-vector attitude weight maps to 0.25*Qq in Lu coordinates.
% The aT input weight is the total-thrust slice of Sun's rotor-thrust
% input cost, with T=m*aT:
%   u' * Win * u = (G^{-1}*mu)' * Win * (G^{-1}*mu)
%                = mu' * Wmu * mu,  Wmu = G^{-T}*Win*G^{-1}.
% Lu has no Omega state in its MPC, so its Omega input penalty uses the
% shared body-rate weight.
[B, ~, ~, ~] = irisAllocationMatrices(par);
G = [-B(1,:); B(2:4,:)];
Win = par.mpc.Qu;
Ginv = G\eye(4);
Wmu = Ginv' * Win * Ginv;
par.lu.Q = blkdiag(par.mpc.Qp, par.mpc.Qv, 0.25*par.mpc.Qq);
par.lu.P = par.lu.Q;
par.lu.R = blkdiag(par.m^2*Wmu(1,1), par.mpc.QOmega);
par.lu.R = 0.5*(par.lu.R + par.lu.R');

% [4] S. Sun, A. Romero, P. Foehn, E. Kaufmann, and D. Scaramuzza, “A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight,” Feb. 23, 2022, arXiv: arXiv:2109.01365. Accessed: May 27, 2022. [Online]. Available: http://arxiv.org/abs/2109.01365
% Sun et al. 2022, Table I controller parameters. The plant/allocation layer
% uses the benchmark geometry; Sun NMPC/DFBC inputs are individual rotor
% thrusts u = [u1;u2;u3;u4], as in Eq. (4)-(12).
par.sun.N = par.mpc.N;
par.sun.dt = par.mpc.dt;
par.sun.horizon = par.sun.N * par.sun.dt;
par.sun.Qxi = par.mpc.Qp;
par.sun.Qv = par.mpc.Qv;
par.sun.Qq = par.mpc.Qq;
par.sun.QOmega = par.mpc.QOmega;
par.sun.Qu = par.mpc.Qu;
par.sun.QN = blkdiag(par.sun.Qxi, par.sun.Qv, par.sun.Qq, par.sun.QOmega);

% Sun Table I, DFBC column. These are feedback/controller gains, not the
% NMPC objective weights Qxi/Qv/Qq/QOmega above. Eq. (28) uses the quaternion
% reduced-attitude vector qRed, whose small-angle roll/pitch components are
% approximately 0.5*theta. Tal/geometric use full-angle log errors, so use a
% factor of 2 on roll/pitch to keep comparable small-angle stiffness.
par.sun.Kxi = par.Kp;
par.sun.Kv = par.Kv;
par.sun.KqRed = diag([2*par.KR(1,1), 2*par.KR(2,2), 0]);
par.sun.kqYaw = par.KR(3,3);
par.sun.KOmega = par.KOmega;
par.sun.allocationW = diag([0.001, 10, 10, 0.1]);
par.sun.omegaMax = [10; 10; 4];
sunRepoDir = string(fileparts(mfilename('fullpath')));
if strlength(sunRepoDir) == 0
    sunRepoDir = string(pwd);
end

acadosSourceDir = string(getenv("ACADOS_SOURCE_DIR"));
if strlength(acadosSourceDir) == 0
    acadosSourceDir = fullfile(sunRepoDir, ".acados", "acados");
end
par.sun.acadosSourceDir = acadosSourceDir;
par.sun.acadosToolsPath = fullfile(sunRepoDir, "tools");
sunPython = string(getenv("ACADOS_PYTHON"));
if strlength(sunPython) == 0
    projectSunPython = fullfile(sunRepoDir, ".venv", "bin", "python3");
    if exist(projectSunPython, "file") == 2
        sunPython = string(projectSunPython);
    end
end
par.sun.pythonExecutable = sunPython;
par.sun.solvePeriod = par.control.outerPeriod;  % NMPC refresh period Tc = 100 Hz.
par.sun.indiPeriod = par.control.innerPeriod;   % 250 Hz INDI inner-loop refresh.
par.sun.printSolverTiming = false;
par.sun.innerLoopFilterCutoffHz = par.indi.innerLoopFilterCutoffHz;
% d_tau is not modeled as a known input; Sun's INDI loop absorbs it through
% filtered angular-acceleration and rotor-thrust feedback.

% Geometric INDI gains.
% No author-specific reference namespace.
par.geometric_indi.Kp = par.Kp;
par.geometric_indi.Kv = par.Kv;
par.geometric_indi.Ktheta = par.KR;
par.geometric_indi.Komega = par.KOmega;
% Match Tal's INDI practice: filter measured acceleration/rate feedback
% before using it in incremental inversion.
par.geometric_indi.innerLoopFilterCutoffHz = par.indi.innerLoopFilterCutoffHz;
par.geometric_indi.outerLoopFilterCutoffHz = par.indi.outerLoopFilterCutoffHz;

% [3] E. Tal and S. Karaman, “Accurate Tracking of Aggressive Quadrotor Trajectories Using Incremental Nonlinear Dynamic Inversion and Differential Flatness,” IEEE Trans. Contr. Syst. Technol., vol. 29, no. 3, pp. 1203–1218, May 2021, doi: 10.1109/tcst.2020.3001117.
% Tal and Karaman INDI + differential-flatness controller.
% Use controller-specific gains even when their numerical defaults are close
% to the baseline controller; this keeps each paper controller independently
% tunable.
par.tal.Kp = par.Kp;                     % Eq. (17), position term
par.tal.Kv = par.Kv;                     % Eq. (17), velocity term
par.tal.Ka = 0.0*eye(3);                 % Eq. (17), acceleration term
% Eq. (28), attitude error and angular-rate terms.
par.tal.Ktheta = par.KR;
par.tal.Komega = par.KOmega;
% Fig. 4: identical second-order Butterworth LPFs, 30 Hz cutoff.
par.tal.innerLoopFilterCutoffHz = par.indi.innerLoopFilterCutoffHz;
par.tal.outerLoopFilterCutoffHz = par.indi.outerLoopFilterCutoffHz;

% Measurement noise injected only into controller feedback. These are
% deliberately challenging state-estimate RMS levels, not raw sensor noise.
% The plant state x remains true; controllers get xMeas.
par.feedbackNoise.enabled = false;
par.feedbackNoise.seed = 1801248224;
par.feedbackNoise.positionStd = [0.08; 0.08; 0.1];       % [m]
par.feedbackNoise.velocityStd = [0.04; 0.04; 0.04];       % [m/s]
par.feedbackNoise.attitudeStd = deg2rad([0.034; 0.034; 0.068]); % [rad]
par.feedbackNoise.omegaStd = deg2rad([0.01; 0.01; 0.01]);    % [rad/s]

% Additive plant disturbances. main.m defaults to the disturbance benchmark's
% high random_gust case. Set enabled=false for the no-disturbance baseline.
% When enabled, type must be "constant", "sin", or "random".
% The force disturbance is expressed in inertial NED coordinates [N]; the
% moment disturbance is expressed in the body frame [N*m].
par.disturbance.enabled = false;
par.disturbance.type = "random";
par.disturbance.seed = 2042294704;
par.disturbance.sampleTime = par.dt;
% For constant/sin, these are amplitudes. For random, they are stationary
% RMS levels of a first-order Gauss-Markov gust/load process.
par.disturbance.forceAmp = [0.45; 0.45; 0.22];      % [Fx; Fy; Fz] [N]
par.disturbance.momentAmp = [0.080; 0.080; 0.008];  % [Mx; My; Mz] [N*m]
par.disturbance.forceFreq = [0.17; 0.23; 0.31];   % sinusoid frequencies [Hz]
par.disturbance.momentFreq = [0.19; 0.29; 0.37];  % sinusoid frequencies [Hz]
par.disturbance.forcePhase = [0; 1*pi/3; 2*pi/3];
par.disturbance.momentPhase = [pi/4; 3*pi/4; 5*pi/4];
par.disturbance.forceTau = [1.5; 1.5; 1.0];        % random correlation time [s]
par.disturbance.momentTau = [0.35; 0.35; 0.25];    % random correlation time [s]
par.disturbance.startTime = 0;
par.disturbance.endTime = inf;

% Initial condition
par.startOnReference = true;

% Post-simulation plotting
par.enablePlots = true;
par.saveResults = false;
par.saveFigures = false;
par.saveMat = false;
par.resultDir = fullfile(pwd, "results", "main");
par.plotStateDetail = true;        % Optional state/component tracking detail.
par.plotBodyAxes = false;
par.plotBodyAxesEvery = 1;          % seconds, only used when plotBodyAxes=true.
par.plotBodyAxisScale = 0.3;        % meters, only used when plotBodyAxes=true.
par.plotBodyAxesPoseSource = "actual"; % "actual" or "desired"
par.animationSpeed = 1;       % 1.0 = real time
par.animationFrameDt = 0.02;    % seconds
par.animationPoseSource = "actual";  % "actual" or "desired"
par.animationBodyAxisScale = 0.3;    % meters

if ~isempty(fieldnames(parOverride__))
    par = mergeStructRecursive(par, parOverride__);
end
par = finalizeAeroConfig(par);
par = finalizeActuatorModel(par);
par = finalizePX4CascadedGains(par);
par = finalizeMPCConfig(par);

Aa = [zeros(3), eye(3), zeros(3);
      zeros(3), zeros(3), zeros(3);
      eye(3),  zeros(3), zeros(3)];
Ba = [zeros(3); eye(3)/par.m; zeros(3)];
Ha = [Aa, -Ba*(par.johnson.Wf\Ba');
      -par.johnson.We, -Aa'];
[Va, Da] = eig(Ha);
Va = Va(:, real(diag(Da)) < 0);
Pa = real(Va(10:18,:)/Va(1:9,:));
Pa = 0.5*(Pa + Pa');
par.johnson.Klqr = par.johnson.Wf\(Ba'*Pa);

%% ========================================================================
%% 1. Build trajectory
traj = makeTrajectory(par);
par.Tend = traj.Tend;

%% ========================================================================
%% 2. Initial state
ref0 = traj.eval(0);
% Initial reference attitude:
%   T0*b3d0 = m*(g*e3-a_d0), b3d0 = T0*b3d0/||T0*b3d0||,
% then combine that thrust axis with the initial yaw heading.
thrustAxisForce0 = par.m*(par.g*par.e3 - ref0.a);
if norm(thrustAxisForce0) < 1e-9 %第一类奇异点
    thrustAxisForce0 = par.m*par.g*par.e3;
end
b3d0 = thrustAxisForce0/norm(thrustAxisForce0);
xC0 = [cos(ref0.psi); sin(ref0.psi); 0];
if norm(cross(b3d0, xC0)) < 1e-8 %第二类奇异点
    xC0 = [0; 1; 0];
end
R0 = attitudeFromUnitBodyZAndHeading(b3d0, xC0);

if par.startOnReference
    x.p = ref0.p;
    x.v = ref0.v;
    x.R = R0;
    x.Omega = zeros(3,1);
else
    x.p = [0;0;0];
    x.v = [0;0;0];
    x.R = eye(3);
    x.Omega = zeros(3,1);
end

%% ========================================================================
%% 3. Logs
time = 0:par.dt:par.Tend;
N = numel(time);
par = prepareRandomModels(par, time);

log.p = zeros(3,N);
log.v = zeros(3,N);
log.pMeas = zeros(3,N);
log.vMeas = zeros(3,N);
log.pd = zeros(3,N);
log.vd = zeros(3,N);
log.ad = zeros(3,N);
log.vPlotD = zeros(3,N);
log.aPlotD = zeros(3,N);

log.R = zeros(3,3,N);
log.RMeas = zeros(3,3,N);
log.Rd = zeros(3,3,N);
log.Omega = zeros(3,N);
log.OmegaMeas = zeros(3,N);
log.OmegaD = nan(3,N);
log.alphaD = nan(3,N);
log.OmegaDProvided = false(1,N);
log.alphaDProvided = false(1,N);

log.euler = zeros(3,N);
log.eulerD = zeros(3,N);

log.T = zeros(1,N);
log.tau = zeros(3,N);
log.actuator = nan(numel(par.allocation.uMin),N);
log.aeroForce = zeros(3,N);
log.forceDist = zeros(3,N);
log.momentDist = zeros(3,N);
log.sunNMPCCached = false(1,N);
log.sunNMPCSolved = false(1,N);
log.sunNMPCStatusCode = zeros(1,N);
log.sunNMPCExitflag = nan(1,N);
log.sunNMPCSolveTime = nan(1,N);

%% ========================================================================
%% 4. Simulation loop
for k = 1:N
    t = time(k);

    ref = traj.eval(t);
    xMeas = measuredStateAtIndex(x, k, par);
    u = controller(xMeas, ref, traj, t, par);

    log.p(:,k) = x.p;
    log.v(:,k) = x.v;
    log.pMeas(:,k) = xMeas.p;
    log.vMeas(:,k) = xMeas.v;
    log.pd(:,k) = ref.p;
    log.vd(:,k) = ref.v;
    log.ad(:,k) = ref.a;
    log.vPlotD(:,k) = ref.v;
    log.aPlotD(:,k) = ref.a;
    if isfield(u, 'vPlotD')
        log.vPlotD(:,k) = u.vPlotD;
    end
    if isfield(u, 'aPlotD')
        log.aPlotD(:,k) = u.aPlotD;
    end

    log.R(:,:,k) = x.R;
    log.RMeas(:,:,k) = xMeas.R;
    log.Rd(:,:,k) = u.Rd;
    log.Omega(:,k) = x.Omega;
    log.OmegaMeas(:,k) = xMeas.Omega;
    if isfield(u, 'OmegaD')
        log.OmegaD(:,k) = u.OmegaD;
        log.OmegaDProvided(k) = true;
    end
    if isfield(u, 'alphaD')
        log.alphaD(:,k) = u.alphaD;
        log.alphaDProvided(k) = true;
    end

    log.euler(:,k) = flip(rotm2eul(x.R, "ZYX")).';
    log.eulerD(:,k) = flip(rotm2eul(u.Rd, "ZYX")).';

    log.T(k) = u.T;
    log.tau(:,k) = u.tau;
    if isfield(u, 'actuator')
        log.actuator(:,k) = u.actuator(:);
    end
    log.aeroForce(:,k) = x.R*sunAeroForceBody(x.R, x.v, par);
    [log.forceDist(:,k), log.momentDist(:,k)] = disturbanceAtTime(t, par);

    if isfield(u, 'sunNMPCSolved')
        log.sunNMPCCached(k) = u.sunNMPCCached;
        log.sunNMPCSolved(k) = u.sunNMPCSolved;
        log.sunNMPCStatusCode(k) = u.sunNMPCStatusCode;
        log.sunNMPCExitflag(k) = u.sunNMPCExitflag;
        log.sunNMPCSolveTime(k) = u.sunNMPCSolveTime;
    end

    x = stepModel(x, u, par, t);
end

if any(par.controllerName == ["sun_nmpc", "sun_nmpc_indi"])
    nCached = nnz(log.sunNMPCCached);
    nSolverFailures = nnz(log.sunNMPCStatusCode ~= 0 & ~log.sunNMPCCached);
    fprintf(['sun_nmpc solve status: %d optimized solves, ' ...
        '%d cached steps, %d nonzero statuses, mean solve %.3f s.\n'], ...
        nnz(log.sunNMPCSolved), nCached, nSolverFailures, ...
        mean(log.sunNMPCSolveTime(log.sunNMPCSolved), 'omitnan'));
end

%% ========================================================================
%% 5. Plot
if par.saveResults
    par.resultDir = saveMainRunResults(time, log, par, traj);
end

if par.enablePlots
    plot_main(time, log, par, traj, ...
        'OutputDir', fullfile(par.resultDir, 'figures'), ...
        'SavePlots', par.saveFigures, ...
        'PlotStateDetail', par.plotStateDetail);
end

%% ========================================================================
%% Trajectory factory
function traj = makeTrajectory(par)

    shape = trajectoryShape(par);
    traj.name = string(par.trajName);
    traj.Tend = par.Tend;

    switch par.trajName

        case "figure8_horizontal"
            cfg = makeFigure8HorizontalParams(shape);
            traj.eval = @(t) evalFigure8Horizontal(t, cfg, false);
            traj.evalPredict = @(t) evalFigure8Horizontal(t, cfg, true);

        case "figure8_vertical"
            cfg = makeFigure8VerticalParams(shape);
            traj.eval = @(t) evalFigure8Vertical(t, cfg, false);
            traj.evalPredict = @(t) evalFigure8Vertical(t, cfg, true);

        case "helix_flip"
            cfg = makeHelixFlipParams(shape);
            traj.eval = @(t) evalHelixFlip(t, cfg, false);
            traj.evalPredict = @(t) evalHelixFlip(t, cfg, true);

        case "helix_flip_y"
            cfg = makeHelixFlipYParams(shape);
            traj.eval = @(t) evalHelixFlipY(t, cfg, false);
            traj.evalPredict = @(t) evalHelixFlipY(t, cfg, true);

        case "flip_loop_sine"
            cfg = makeHelixFlipParams(shape);
            traj.eval = @(t) evalFlipLoop(t, cfg, false);
            traj.evalPredict = @(t) evalFlipLoop(t, cfg, true);

        case "fast_circle"
            cfg = makeFastCircleParams(shape);
            traj.eval = @(t) evalFastCircle(t, cfg, false);
            traj.evalPredict = @(t) evalFastCircle(t, cfg, true);

        otherwise
            error("Unknown trajectory name.");
    end

    traj.name = trajectoryNameWithOmega(traj.name, cfg.omega);
end

function name = trajectoryNameWithOmega(name, omega)

    switch omega.mode
        case "fixed"
            name = name + "_omegaFixed_" + string(omega.value);
        case {"linear", "quadratic"}
            name = name + "_omega" + upperFirst(omega.mode) ...
                + "_" + string(omega.range(1)) + "_" + string(omega.range(2));
    end
end

function text = upperFirst(text)

    text = string(text);
    text = upper(extractBefore(text, 2)) + extractAfter(text, 1);
end

function omega = resolveTrajectoryOmega(shape)

    raw = shape.omega;
    omegaName = shape.paramName + ".omega";
    mode = lower(string(getStructField(raw, 'mode', "fixed")));

    switch mode
        case {"fixed", "omega_fixed"}
            value = double(getStructField(raw, 'value', 1));
            validateOmegaValues(value, omegaName + ".value");
            omega.mode = "fixed";
            omega.value = value;
            omega.range = [value, value];

        case {"linear", "omega_linear", "quadratic", "omega_quadratic"}
            mode = erase(mode, "omega_");
            range = vectorNParam(getStructField(raw, 'range', [1, 1]), 2, ...
                omegaName + ".range");
            validateOmegaValues(range, omegaName + ".range");
            omega.mode = mode;
            omega.value = range(2);
            omega.range = range(:).';

        otherwise
            error('Unknown %s.mode "%s".', omegaName, mode);
    end

    if ~isempty(fieldnames(shape.legacyProgress))
        omega = omegaFromLegacyProgress(shape.legacyProgress, omega);
        return;
    end
end

function omega = omegaFromLegacyProgress(progress, baseOmega)

    mode = lower(string(getStructField(progress, 'mode', "speed_fixed")));
    baseRange = baseOmega.range(:).';
    baseTerminal = max(baseOmega.value, eps);

    switch mode
        case {"speed_fixed", "scale_fixed"}
            scale = double(getStructField(progress, 'speed', ...
                getStructField(progress, 'scale', 1)));
            value = scale*baseTerminal;
            validateOmegaValues(value, 'par.progress.speed/scale');
            omega.mode = "fixed";
            omega.value = value;
            omega.range = [value, value];

        case {"speed_linear", "scale_linear"}
            scaleRange = getStructField(progress, 'speedRange', ...
                getStructField(progress, 'scaleRange', [1, 1]));
            scaleRange = vectorNParam(scaleRange, 2, ...
                'par.progress.speedRange/scaleRange');
            range = scaleRange(:).' .* baseRange;
            validateOmegaValues(range, 'par.progress.speedRange/scaleRange');
            omega.mode = "linear";
            omega.value = range(2);
            omega.range = range(:).';

        otherwise
            error('Unknown legacy par.progress.mode "%s".', mode);
    end
end

function validateOmegaValues(values, name)

    if any(~isfinite(values)) || any(values < 0)
        error("%s must contain finite nonnegative angular frequencies.", name);
    end
end

function [theta, thetaDot, thetaDDot, theta3, theta4] = trajectoryTheta( ...
        t, cfg, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    [theta, thetaDot, thetaDDot, theta3, theta4] = scheduledAngularState( ...
        t, cfg.omega, cfg.Tend, allowPredict);
    theta = cfg.theta0 + theta;
end

function [tau, tauDot] = trajectoryLinearTime(t, Tend, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    if allowPredict
        tau = max(t, 0);
        tauDot = double(t >= 0);
    else
        tau = min(max(t, 0), Tend);
        tauDot = double(t >= 0 && t <= Tend);
    end
end

function [theta, thetaDot, thetaDDot, theta3, theta4] = ...
        scheduledAngularState(t, omega, Tend, allowPredict)

    if nargin < 4
        allowPredict = false;
    end

    Tend = max(double(Tend), eps);

    if allowPredict && t > Tend
        [thetaEnd, omegaEnd] = scheduledAngularState(Tend, omega, Tend, false);
        theta = thetaEnd + omegaEnd*(t - Tend);
        thetaDot = omegaEnd;
        thetaDDot = 0;
        theta3 = 0;
        theta4 = 0;
        return;
    end

    outside = t < 0 || t > Tend;
    t = min(max(t, 0), Tend);

    switch omega.mode
        case "fixed"
            theta = omega.value*t;
            thetaDot = omega.value;
            thetaDDot = 0;
            theta3 = 0;
            theta4 = 0;

        case "linear"
            omega0 = omega.range(1);
            omega1 = omega.range(2);
            omegaDelta = omega1 - omega0;
            theta = omega0*t + 0.5*omegaDelta*t^2/Tend;
            thetaDot = omega0 + omegaDelta*t/Tend;
            thetaDDot = omegaDelta/Tend;
            theta3 = 0;
            theta4 = 0;

        case "quadratic"
            omega0 = omega.range(1);
            omega1 = omega.range(2);
            omegaDelta = omega1 - omega0;
            theta = omega0*t + omegaDelta*t^3/(3*Tend^2);
            thetaDot = omega0 + omegaDelta*(t/Tend)^2;
            thetaDDot = 2*omegaDelta*t/Tend^2;
            theta3 = 2*omegaDelta/Tend^2;
            theta4 = 0;

        otherwise
            error('Unknown angular-frequency schedule "%s".', omega.mode);
    end

    if outside && ~allowPredict
        thetaDot = 0;
        thetaDDot = 0;
        theta3 = 0;
        theta4 = 0;
    end
end

function shape = trajectoryShape(par)

    trajRoot = getStructField(par, 'traj', struct());
    shape.g = par.g;
    shape.Tend = max(double(par.Tend), eps);
    [shape.params, shape.paramName] = trajectoryParamsForName(par, trajRoot);
    shape.omega = getStructField(shape.params, 'omega', ...
        getStructField(trajRoot, 'omega', struct()));
    shape.legacyProgress = getStructField(par, 'progress', struct());

end

function [trajPar, paramName] = trajectoryParamsForName(par, trajRoot)

    fieldName = matlab.lang.makeValidName(char(par.trajName));
    paramName = "par.traj." + string(fieldName);

    if ~isfield(trajRoot, fieldName) || ~isstruct(trajRoot.(fieldName))
        error("%s must be defined as a struct.", paramName);
    end

    trajPar = trajRoot.(fieldName);

    if isfield(trajRoot, 'omega')
        if isfield(trajPar, 'omega') && isstruct(trajPar.omega)
            trajPar.omega = mergeStructRecursive(trajRoot.omega, trajPar.omega);
        else
            trajPar.omega = trajRoot.omega;
        end
    end
end

function value = positiveScalarParam(value, name)

    value = double(value);

    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error("%s must be a finite positive scalar.", name);
    end
end

function value = finiteScalarParam(value, name)

    value = double(value);

    if ~isscalar(value) || ~isfinite(value)
        error("%s must be a finite scalar.", name);
    end
end

function dst = mergeStructRecursive(dst, src)

    names = fieldnames(src);

    for i = 1:numel(names)
        name = names{i};

        if isstruct(src.(name)) ...
                && isfield(dst, name) ...
                && isstruct(dst.(name))
            dst.(name) = mergeStructRecursive(dst.(name), src.(name));
        else
            dst.(name) = src.(name);
        end
    end
end

function par = finalizeAeroConfig(par)

    if ~isfield(par, 'aero')
        par.aero = struct();
    end
    par.aero.enabled = logical(getStructField(par.aero, 'enabled', true));
    par.aero.kd = getStructField(par.aero, 'kd', [0.26; 0.28; 0.42]);
    par.aero.kd = par.aero.kd(:);
    if numel(par.aero.kd) ~= 3
        error("par.aero.kd must be a 3-vector.");
    end
    par.aero.kh = double(getStructField(par.aero, 'kh', 0.01));

    % Keep Sun NMPC/DFBC on the same identified drag model.
    par.sun.aero = par.aero;
end

function par = finalizeActuatorModel(par)

    par.allocation.method = lower(string( ...
        getStructField(par.allocation, 'method', "wls")));
    if ~any(par.allocation.method == ["wls", "pinv"])
        error('par.allocation.method must be "wls" or "pinv".');
    end

    if ~isfield(par.allocation, 'gamma') || par.allocation.gamma <= 0
        par.allocation.gamma = 1e6;
    end

    par.allocation.uMin = par.allocation.uMin(:);
    par.allocation.uMax = par.allocation.uMax(:);

    [par.allocation.B, par.allocation.B_px4, par.allocation.B_px4_norm,par.allocation.D_px4] = ...
        irisAllocationMatrices(par);

    par.Tmax = allocationForceLimit(par);
    par.tauMax = allocationMomentLimits(par);

    % Keep Sun NMPC/DFBC parameters on the same actuator model. This is a
    % compatibility mirror; par.allocation is the source of truth.
    par.sun.uMin = par.allocation.uMin;
    par.sun.uMax = par.allocation.uMax;
end

function par = finalizePX4CascadedGains(par)

    % PX4 gain equivalence used below:
    %
    %   benchmark translational feedback:
    %       a_fb = Kp*e_p + Kv*e_v
    %
    %   PX4 translational feedback:
    %       v_sp = v_ref + K_pos*e_p
    %       a_fb = K_vel*(v_sp - v)
    %            = K_vel*K_pos*e_p + K_vel*e_v
    %
    %   Exact diagonal match:
    %       K_vel = diag(Kv)
    %       K_pos = diag(Kp)./diag(Kv)
    %
    %   benchmark attitude/rate feedback:
    %       tau_fb = J*(KR*e_R + KOmega*(Omega_ref - Omega))
    %
    %   PX4 attitude/rate feedback in physical torque units:
    %       Omega_sp = K_att*e_R + Omega_ff
    %       tau_fb = S_tau*K_rate*(Omega_sp - Omega)
    %              = S_tau*K_rate*K_att*e_R - S_tau*K_rate*Omega
    %
    %   Exact diagonal match:
    %       K_att  = diag(KR)./diag(KOmega)
    %       K_rate = diag(S_tau)\diag(J*KOmega)
    %
    % where S_tau maps PX4 normalized torque to physical body torque for the
    % current Iris allocation matrix.
    % PX4 gain equivalence used below:
    %
    %   benchmark translational feedback:
    %       a_fb = Kp*e_p + Kv*e_v
    %
    %   PX4 translational feedback:
    %       v_sp = v_ref + K_pos*e_p
    %       a_fb = K_vel*(v_sp - v)
    %            = K_vel*K_pos*e_p + K_vel*e_v
    %
    %   Exact diagonal match:
    %       K_vel = diag(Kv)
    %       K_pos = diag(Kp)./diag(Kv)
    %
    %   benchmark attitude/rate feedback:
    %       tau_fb = J*(KR*e_R + KOmega*(Omega_ref - Omega))
    %
    %   PX4 attitude/rate feedback in physical torque units:
    %       Omega_sp = K_att*e_R + Omega_ff
    %       tau_fb = S_tau*K_rate*(Omega_sp - Omega)
    %              = S_tau*K_rate*K_att*e_R - S_tau*K_rate*Omega
    %
    %   Exact diagonal match:
    %       K_att  = diag(KR)./diag(KOmega)
    %       K_rate = diag(S_tau)\diag(J*KOmega)
    %
    % where S_tau maps PX4 normalized torque to physical body torque for the
    % current Iris allocation matrix.
    px4PhysicalFromNorm = par.allocation.B_px4 / par.allocation.B_px4_norm;
    torqueScale = px4PhysicalFromNorm(2:4, 2:4); % inv(torqueScale)=D_px4

    gainSource = string(getStructField(par.px4_iris, 'gainSource', "manual"));
    if gainSource == "benchmark_equivalent"
        KpDiag = diag(par.Kp);
        KvDiag = diag(par.Kv);
        KRDiag = diag(par.KR);
        KOmegaDiag = diag(par.KOmega);

        par.px4_iris.posP = KpDiag ./ KvDiag;
        par.px4_iris.velP = KvDiag;
        par.px4_iris.velI = zeros(3,1);
        par.px4_iris.velD = zeros(3,1);

        % In this MATLAB PX4 branch attP is already the effective
        % axis-angle gain. Keep yawWeight at 1 so the value is transparent.
        par.px4_iris.yawWeight = 1.0;
        par.px4_iris.attP = KRDiag ./ KOmegaDiag;

        KomegaPhysical = par.J * par.KOmega;
        ratePMatrix = torqueScale \ KomegaPhysical;
        par.px4_iris.rateP = diag(ratePMatrix);
        par.px4_iris.rateI = zeros(3,1);
        par.px4_iris.rateD = zeros(3,1);
        par.px4_iris.rateFF = zeros(3,1);
        par.px4_iris.rateIntLimit = inf(3,1);
    end

    par.px4_iris.posP = par.px4_iris.posP(:);
    par.px4_iris.velP = par.px4_iris.velP(:);
    par.px4_iris.velI = par.px4_iris.velI(:);
    par.px4_iris.velD = par.px4_iris.velD(:);
    par.px4_iris.rateP = par.px4_iris.rateP(:);
    par.px4_iris.rateI = par.px4_iris.rateI(:);
    par.px4_iris.rateD = par.px4_iris.rateD(:);
    par.px4_iris.rateFF = par.px4_iris.rateFF(:);
    par.px4_iris.rateI = par.px4_iris.rateI(:);
    par.px4_iris.rateD = par.px4_iris.rateD(:);
    par.px4_iris.rateFF = par.px4_iris.rateFF(:);
    par.px4_iris.attP = par.px4_iris.attP(:);
    KomegaEq = torqueScale * diag(par.px4_iris.rateP);

    par.px4_iris.normalizedTorqueScale = torqueScale;
    par.px4_iris.KOmegaEquivalent = KomegaEq;
    par.px4_iris.KREquivalent = KomegaEq * diag(par.px4_iris.attP);
end

function par = finalizeMPCConfig(par)

    if ~isfield(par, 'control')
        par.control = struct();
    end
    if ~isfield(par.control, 'outerPeriod') || par.control.outerPeriod <= 0
        par.control.outerPeriod = 0.01;
    end
    if ~isfield(par.control, 'innerPeriod') || par.control.innerPeriod <= 0
        par.control.innerPeriod = 1/250;
    end
    if ~isfield(par.sun, 'solvePeriod') || par.sun.solvePeriod <= 0
        par.sun.solvePeriod = par.control.outerPeriod;
    end
    if ~isfield(par.sun, 'indiPeriod') || par.sun.indiPeriod <= 0
        par.sun.indiPeriod = par.control.innerPeriod;
    end
    par.sun.horizon = par.sun.N * par.sun.dt;

    if ~isfield(par.lu, 'dt') || par.lu.dt <= 0
        error("par.lu.dt must be positive for Lu on-manifold MPC.");
    end
    if ~isfield(par.lu, 'solvePeriod') || par.lu.solvePeriod <= 0
        par.lu.solvePeriod = par.lu.dt;
    end
    if ~isfield(par.lu, 'ratePeriod') || par.lu.ratePeriod <= 0
        par.lu.ratePeriod = par.dt;
    end
    par.lu.omegaMax = par.px4_iris.rateLimit(:);
    par.lu.horizon = par.lu.N * par.lu.dt;
end

function tauMax = allocationMomentLimits(par)

    B = par.allocation.B;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    tauMax = zeros(3,1);

    for i = 1:3
        row = B(i+1,:)';
        tauHi = sum(max(row.*lb, row.*ub));
        tauLo = sum(min(row.*lb, row.*ub));
        tauMax(i) = max(abs([tauLo, tauHi]));
    end
end

function Tmax = allocationForceLimit(par)

    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    row = -par.allocation.B(1,:)';
    Tmax = sum(max(row.*lb, row.*ub));
end

function [B, B_px4, B_px4_norm,D_px4] = irisAllocationMatrices(par)

    % This is the control-allocation construction from iris.m, kept local so
    % main.m derives standard and PX4 matrices from the same Gazebo Iris model.
    CT = par.CT;
    kappa = par.kappa;
    pos = par.pos;
    axis = par.axis;
    KM = kappa*par.spin(:);

    B2 = zeros(6,4);

    for i = 1:4
        r = pos(i,:)';
        moment = cross(r, axis) - KM(i)*axis;
        force = axis;
        B2(:,i) = [moment; force];
    end

    B3 = zeros(6,4); % B2 * T_max = B3

    for i = 1:4
        r = pos(i,:)';
        moment = CT*cross(r, axis) - CT*KM(i)*axis;
        force = CT*axis;
        B3(:,i) = [moment; force];
    end

    [D3, B3_norm] = px4_normalize_B(B3, true);
    
    D_px4 = diag([D3(6,6),D3(1,1),D3(2,2),D3(3,3)]);
    B = [B2(6,:); B2(1:3,:)];
    B_px4 = [B3(6,:); B3(1:3,:)];
    B_px4_norm = [B3_norm(6,:); B3_norm(1:3,:)];
    D_tau_px4 = D3(1:3,1:3); %  
    D_T_px4 = D3(4:6,4:6); % D_T_px4= 1/(4*T_max),  h=m*g/(4*T_max), h/g=m/(4*T_max)

end

function [forceDist, momentDist] = disturbanceAtTime(t, par)

    forceDist = zeros(3,1);
    momentDist = zeros(3,1);

    if ~par.disturbance.enabled
        return;
    end

    d = par.disturbance;

    if t < d.startTime || t > d.endTime
        return;
    end

    forceAmp = vector3(d.forceAmp);
    momentAmp = vector3(d.momentAmp);

    switch string(d.type)
        case "constant"
            forceDist = forceAmp;
            momentDist = momentAmp;

        case "sin"
            forceFreq = vector3(d.forceFreq);
            momentFreq = vector3(d.momentFreq);
            forcePhase = vector3(d.forcePhase);
            momentPhase = vector3(d.momentPhase);

            forceDist = forceAmp .* sin(2*pi*forceFreq*t + forcePhase);
            momentDist = momentAmp .* sin(2*pi*momentFreq*t + momentPhase);

        case "random"
            if ~isfield(d, 'random') ...
                    || ~isfield(d.random, 'force') ...
                    || ~isfield(d.random, 'moment')
                error("Random disturbance was not initialized.");
            end
            forceDist = sampleUniformSeries(t, d.random.time, d.random.force);
            momentDist = sampleUniformSeries(t, d.random.time, d.random.moment);

        otherwise
            error("Unknown disturbance type.");
    end
end

function par = prepareRandomModels(par, time)

    if isfield(par, 'feedbackNoise') && par.feedbackNoise.enabled
        par.feedbackNoise.samples = generateFeedbackNoiseSamples( ...
            par.feedbackNoise, numel(time));
    end

    if isfield(par, 'disturbance') ...
            && par.disturbance.enabled ...
            && string(par.disturbance.type) == "random"
        par.disturbance.random = generateRandomDisturbanceSamples( ...
            par.disturbance, time);
    end
end

function samples = generateFeedbackNoiseSamples(noise, nSamples)

    stream = RandStream('mt19937ar', ...
        'Seed', normalizedSeed(getStructField(noise, 'seed', 0)));

    samples.p = vector3Param(getStructField(noise, ...
        'positionStd', zeros(3,1)), 'feedbackNoise.positionStd') ...
        .* randn(stream, 3, nSamples);
    samples.v = vector3Param(getStructField(noise, ...
        'velocityStd', zeros(3,1)), 'feedbackNoise.velocityStd') ...
        .* randn(stream, 3, nSamples);
    samples.attitude = vector3Param(getStructField(noise, ...
        'attitudeStd', zeros(3,1)), 'feedbackNoise.attitudeStd') ...
        .* randn(stream, 3, nSamples);
    samples.omega = vector3Param(getStructField(noise, ...
        'omegaStd', zeros(3,1)), 'feedbackNoise.omegaStd') ...
        .* randn(stream, 3, nSamples);
end

function samples = generateRandomDisturbanceSamples(d, time)

    if numel(time) >= 2
        defaultSampleTime = time(2) - time(1);
    else
        defaultSampleTime = 0.01;
    end

    sampleTime = double(getStructField(d, 'sampleTime', defaultSampleTime));
    if ~isfinite(sampleTime) || sampleTime <= 0
        sampleTime = defaultSampleTime;
    end

    tEnd = max(time(end), 0) + sampleTime;
    samples.time = 0:sampleTime:tEnd;
    nSamples = numel(samples.time);

    stream = RandStream('mt19937ar', ...
        'Seed', normalizedSeed(getStructField(d, 'seed', 0)));

    forceAmp = vector3Param(getStructField(d, ...
        'forceAmp', zeros(3,1)), 'disturbance.forceAmp');
    momentAmp = vector3Param(getStructField(d, ...
        'momentAmp', zeros(3,1)), 'disturbance.momentAmp');
    forceTau = vector3Param(getStructField(d, ...
        'forceTau', ones(3,1)), 'disturbance.forceTau');
    momentTau = vector3Param(getStructField(d, ...
        'momentTau', 0.25*ones(3,1)), 'disturbance.momentTau');

    samples.force = forceAmp .* gaussMarkovUnitSamples( ...
        stream, nSamples, sampleTime, forceTau);
    samples.moment = momentAmp .* gaussMarkovUnitSamples( ...
        stream, nSamples, sampleTime, momentTau);
end

function values = gaussMarkovUnitSamples(stream, nSamples, sampleTime, tau)

    tau = max(tau(:), eps);
    phi = exp(-sampleTime ./ tau);
    sigma = sqrt(max(0, 1 - phi.^2));

    values = zeros(3, nSamples);
    if nSamples == 0
        return;
    end

    values(:,1) = randn(stream, 3, 1);
    for kSample = 2:nSamples
        values(:,kSample) = phi .* values(:,kSample-1) ...
            + sigma .* randn(stream, 3, 1);
    end
end

function xMeas = measuredStateAtIndex(x, k, par)

    xMeas = x;
    if ~isfield(par, 'feedbackNoise') || ~par.feedbackNoise.enabled
        return;
    end

    if ~isfield(par.feedbackNoise, 'samples')
        error("Feedback noise was not initialized.");
    end

    nSamples = size(par.feedbackNoise.samples.p, 2);
    iSample = min(max(1, k), nSamples);

    xMeas.p = x.p + par.feedbackNoise.samples.p(:,iSample);
    xMeas.v = x.v + par.feedbackNoise.samples.v(:,iSample);
    dtheta = par.feedbackNoise.samples.attitude(:,iSample);
    xMeas.R = projectSO3(x.R*expm(hat(dtheta)));
    xMeas.Omega = x.Omega + par.feedbackNoise.samples.omega(:,iSample);
end

function y = sampleUniformSeries(t, time, values)

    if isempty(time)
        y = zeros(size(values, 1), 1);
        return;
    end

    if isscalar(time) || t <= time(1)
        y = values(:,1);
        return;
    end

    if t >= time(end)
        y = values(:,end);
        return;
    end

    sampleTime = time(2) - time(1);
    idx = floor((t - time(1))/sampleTime) + 1;
    idx = min(max(1, idx), numel(time) - 1);
    alpha = (t - time(idx))/sampleTime;
    y = (1 - alpha)*values(:,idx) + alpha*values(:,idx+1);
end

function value = getStructField(s, name, defaultValue)

    if isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end

function M = matrix3(x, name)

    if isscalar(x)
        M = x*eye(3);
    elseif isvector(x) && numel(x) == 3
        M = diag(x(:));
    else
        M = x;
    end

    if ~isequal(size(M), [3, 3])
        error("%s must be scalar, 3-vector, or 3x3.", name);
    end
end

function v = vector3(x)

    v = x(:);

    if numel(v) ~= 3
        error("Value must be 3x1.");
    end
end

function v = vector3Param(x, name)

    if isscalar(x)
        v = repmat(double(x), 3, 1);
    else
        v = x(:);
    end

    if numel(v) ~= 3
        error("%s must be scalar or 3x1.", name);
    end
end

function v = vectorNParam(x, n, name)

    if isscalar(x)
        v = repmat(double(x), n, 1);
    else
        v = x(:);
    end

    if numel(v) ~= n
        error("%s must be scalar or %dx1.", name, n);
    end

    v = double(v);
end

function seed = normalizedSeed(seed)

    seed = double(seed);
    if ~isscalar(seed) || ~isfinite(seed)
        seed = 0;
    end
    seed = mod(round(seed), 2^31 - 1);
end

function ref = completeReferenceDerivatives(ref)

    if ~isfield(ref, 'j')
        ref.j = zeros(3,1);
    end

    if ~isfield(ref, 's')
        ref.s = zeros(3,1);
    end

    if ~isfield(ref, 'psiDot')
        ref.psiDot = 0;
    end

    if ~isfield(ref, 'psiDDot')
        ref.psiDDot = 0;
    end
end

function ref = setHeadingFromVelocity(ref, defaultPsi)

    % Heading law used by the horizontal figure-eight, fast circle, and race
    % track curves:
    %   psi(t) = atan2(v_y(t), v_x(t)).
    % The trajectory evaluator already returns time derivatives, so this helper
    % computes yaw rate and yaw acceleration directly from v/a/j:
    %   psi_dot = (v_x*a_y - v_y*a_x)/(v_x^2 + v_y^2),
    %   psi_ddot = ((v_x*j_y - v_y*j_x)*(v_x^2+v_y^2)
    %               - (v_x*a_y - v_y*a_x)*2*(v_x*a_x+v_y*a_y))
    %              /(v_x^2+v_y^2)^2.
    % If the horizontal speed is zero, psi is undefined and defaultPsi is
    % used with zero yaw derivatives.
    speed2 = ref.v(1)^2 + ref.v(2)^2;

    if speed2 < 1e-10
        % Special handling: yaw is undefined at zero horizontal speed, e.g.
        % hover segments. Keep the requested default heading exactly.
        ref.psi = defaultPsi;
        ref.psiDot = 0;
        ref.psiDDot = 0;
        return;
    end

    num = ref.v(1)*ref.a(2) - ref.v(2)*ref.a(1);
    denDot = 2*(ref.v(1)*ref.a(1) + ref.v(2)*ref.a(2));
    numDot = ref.v(1)*ref.j(2) - ref.v(2)*ref.j(1);

    ref.psi = atan2(ref.v(2), ref.v(1));
    ref.psiDot = num/speed2;
    ref.psiDDot = (numDot*speed2 - num*denDot)/speed2^2;
end

function ref = setConstantHeading(ref, psi)

    % Heading law used by vertical/flip trajectories:
    %   psi(t) = psi_c, psi_dot(t) = psi_ddot(t) = 0.
    ref.psi = psi;
    ref.psiDot = 0;
    ref.psiDDot = 0;
end

function [y, yd, ydd, y3, y4] = trigDerivatives( ...
        amplitude, theta, thetaDot, thetaDDot, theta3, theta4, kind)

    s = sin(theta);
    c = cos(theta);

    switch string(kind)
        case "sin"
            y = amplitude*s;
            yd = amplitude*c*thetaDot;
            ydd = amplitude*(c*thetaDDot - s*thetaDot^2);
            y3 = amplitude*(c*theta3 - 3*s*thetaDot*thetaDDot ...
                - c*thetaDot^3);
            y4 = amplitude*(c*theta4 - 4*s*thetaDot*theta3 ...
                - 3*s*thetaDDot^2 - 6*c*thetaDot^2*thetaDDot ...
                + s*thetaDot^4);

        case "cos"
            y = amplitude*c;
            yd = -amplitude*s*thetaDot;
            ydd = amplitude*(-c*thetaDot^2 - s*thetaDDot);
            y3 = amplitude*(s*thetaDot^3 - 3*c*thetaDot*thetaDDot ...
                - s*theta3);
            y4 = amplitude*(c*thetaDot^4 + 6*s*thetaDot^2*thetaDDot ...
                - 3*c*thetaDDot^2 - 4*c*thetaDot*theta3 ...
                - s*theta4);

        otherwise
            error("Unknown trigonometric derivative kind.");
    end
end

%% ========================================================================
%% Analytic horizontal figure-eight
%   theta = theta0 + integral omega(t) dt,
%   r(t) = [Ax*sin(theta);
%           Ay*sin(2*theta);
%           -Hc].
%   psi(t) follows the horizontal velocity heading.
function cfg = makeFigure8HorizontalParams(shape)

    p = shape.params;
    name = shape.paramName;
    cfg.Ax = positiveScalarParam(getStructField(p, 'Ax', 2.5), name + ".Ax");
    cfg.Ay = positiveScalarParam(getStructField(p, 'Ay', 1.5), name + ".Ay");
    cfg.Hc = positiveScalarParam(getStructField(p, 'Hc', 3.0), name + ".Hc");
    cfg.theta0 = finiteScalarParam(getStructField(p, 'theta0', 0), ...
        name + ".theta0");
    cfg.Tend = shape.Tend;
    cfg.omega = resolveTrajectoryOmega(shape);
end

function ref = evalFigure8Horizontal(t, cfg, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    [theta, thetaDot, thetaDDot, theta3, theta4] = ...
        trajectoryTheta(t, cfg, allowPredict);

    [x, vx, ax, jx, sx] = trigDerivatives( ...
        cfg.Ax, theta, thetaDot, thetaDDot, theta3, theta4, "sin");
    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.Ay, 2*theta, 2*thetaDot, 2*thetaDDot, ...
        2*theta3, 2*theta4, "sin");

    ref.p = [x; y; -cfg.Hc];
    ref.v = [vx; vy; 0];
    ref.a = [ax; ay; 0];
    ref.j = [jx; jy; 0];
    ref.s = [sx; sy; 0];
    ref = setHeadingFromVelocity(ref, 0);
end

%% ========================================================================
%% Analytic vertical figure-eight
%   r(t) = [0;
%           Ay*sin(theta);
%           -Hc - Az*sin(2*theta)].
function cfg = makeFigure8VerticalParams(shape)

    p = shape.params;
    name = shape.paramName;
    cfg.Ay = positiveScalarParam(getStructField(p, 'Ay', 2.5), name + ".Ay");
    cfg.Az = positiveScalarParam(getStructField(p, 'Az', 1.5), name + ".Az");
    cfg.Hc = positiveScalarParam(getStructField(p, 'Hc', 2.5), name + ".Hc");
    cfg.theta0 = finiteScalarParam(getStructField(p, 'theta0', -pi/4), ...
        name + ".theta0");
    cfg.Tend = shape.Tend;
    cfg.omega = resolveTrajectoryOmega(shape);
end

function ref = evalFigure8Vertical(t, cfg, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    [theta, thetaDot, thetaDDot, theta3, theta4] = ...
        trajectoryTheta(t, cfg, allowPredict);

    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.Ay, theta, thetaDot, thetaDDot, theta3, theta4, "sin");
    [zOsc, vz, az, jz, sz] = trigDerivatives( ...
        -cfg.Az, 2*theta, 2*thetaDot, 2*thetaDDot, ...
        2*theta3, 2*theta4, "sin");

    ref.p = [0; y; -cfg.Hc + zOsc];
    ref.v = [0; vy; vz];
    ref.a = [0; ay; az];
    ref.j = [0; jy; jz];
    ref.s = [0; sy; sz];
    ref = setConstantHeading(ref, 0);
end

%% ========================================================================
%% Analytic x-forward helix_flip
%   x = Vx*t, y = Ay*sin(theta), z = -Hc + Az*cos(theta).
% flip_loop_sine uses the same form with Vx=0.
function cfg = makeHelixFlipParams(shape)

    p = shape.params;
    name = shape.paramName;
    cfg.Ay = positiveScalarParam(getStructField(p, 'Ay', 2.0), name + ".Ay");
    cfg.Az = positiveScalarParam(getStructField(p, 'Az', 2.0), name + ".Az");
    cfg.Hc = positiveScalarParam(getStructField(p, 'Hc', 3.0), name + ".Hc");
    cfg.Vx = finiteScalarParam(getStructField(p, 'Vx', 0.0), name + ".Vx");
    cfg.theta0 = finiteScalarParam(getStructField(p, 'theta0', 0), ...
        name + ".theta0");
    cfg.Tend = shape.Tend;
    cfg.omega = resolveTrajectoryOmega(shape);
end

function cfg = makeHelixFlipYParams(shape)

    p = shape.params;
    name = shape.paramName;
    cfg.Ax = positiveScalarParam(getStructField(p, 'Ax', 2.0), name + ".Ax");
    cfg.Az = positiveScalarParam(getStructField(p, 'Az', 2.0), name + ".Az");
    cfg.Hc = positiveScalarParam(getStructField(p, 'Hc', 3.0), name + ".Hc");
    cfg.Vy = finiteScalarParam(getStructField(p, 'Vy', 0.0), name + ".Vy");
    cfg.theta0 = finiteScalarParam(getStructField(p, 'theta0', 0), ...
        name + ".theta0");
    cfg.Tend = shape.Tend;
    cfg.omega = resolveTrajectoryOmega(shape);
end

function ref = evalHelixFlip(t, cfg, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    [theta, thetaDot, thetaDDot, theta3, theta4] = ...
        trajectoryTheta(t, cfg, allowPredict);

    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.Ay, theta, thetaDot, thetaDDot, theta3, theta4, "sin");
    [zOsc, vz, az, jz, sz] = trigDerivatives( ...
        cfg.Az, theta, thetaDot, thetaDDot, theta3, theta4, "cos");

    [tau, tauDot] = trajectoryLinearTime(t, cfg.Tend, allowPredict);
    x = cfg.Vx*tau;
    vx = cfg.Vx*tauDot;
    ax = 0;
    jx = 0;
    sx = 0;

    ref.p = [x; y; -cfg.Hc + zOsc];
    ref.v = [vx; vy; vz];
    ref.a = [ax; ay; az];
    ref.j = [jx; jy; jz];
    ref.s = [sx; sy; sz];
    ref = setConstantHeading(ref, 0);
end

function ref = evalHelixFlipY(t, cfg, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    [theta, thetaDot, thetaDDot, theta3, theta4] = ...
        trajectoryTheta(t, cfg, allowPredict);

    [x, vx, ax, jx, sx] = trigDerivatives( ...
        cfg.Ax, theta, thetaDot, thetaDDot, theta3, theta4, "sin");
    [zOsc, vz, az, jz, sz] = trigDerivatives( ...
        cfg.Az, theta, thetaDot, thetaDDot, theta3, theta4, "cos");

    [tau, tauDot] = trajectoryLinearTime(t, cfg.Tend, allowPredict);
    y = cfg.Vy*tau;
    vy = cfg.Vy*tauDot;
    ay = 0;
    jy = 0;
    sy = 0;

    ref.p = [x; y; -cfg.Hc + zOsc];
    ref.v = [vx; vy; vz];
    ref.a = [ax; ay; az];
    ref.j = [jx; jy; jz];
    ref.s = [sx; sy; sz];
    ref = setConstantHeading(ref, 0);
end

function ref = evalFlipLoop(t, cfg, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    [theta, thetaDot, thetaDDot, theta3, theta4] = ...
        trajectoryTheta(t, cfg, allowPredict);

    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.Ay, theta, thetaDot, thetaDDot, theta3, theta4, "sin");
    [zOsc, vz, az, jz, sz] = trigDerivatives( ...
        cfg.Az, theta, thetaDot, thetaDDot, theta3, theta4, "cos");

    [tau, tauDot] = trajectoryLinearTime(t, cfg.Tend, allowPredict);
    x = cfg.Vx*tau;
    vx = cfg.Vx*tauDot;

    ref.p = [x; y; -cfg.Hc + zOsc];
    ref.v = [vx; vy; vz];
    ref.a = [0; ay; az];
    ref.j = [0; jy; jz];
    ref.s = [0; sy; sz];
    ref = setConstantHeading(ref, 0);
end

%% ========================================================================
%% Analytic fast horizontal circle
%   r(t) = [Ax*cos(theta); Ay*sin(theta); -Hc].
%   psi(t) follows the horizontal velocity heading.
function cfg = makeFastCircleParams(shape)

    p = shape.params;
    name = shape.paramName;
    cfg.Ax = positiveScalarParam(getStructField(p, 'Ax', 3.0), name + ".Ax");
    cfg.Ay = positiveScalarParam(getStructField(p, 'Ay', 3.0), name + ".Ay");
    cfg.Hc = positiveScalarParam(getStructField(p, 'Hc', 3.0), name + ".Hc");
    cfg.theta0 = finiteScalarParam(getStructField(p, 'theta0', 0), ...
        name + ".theta0");
    cfg.Tend = shape.Tend;
    cfg.omega = resolveTrajectoryOmega(shape);
end

function ref = evalFastCircle(t, cfg, allowPredict)

    if nargin < 3
        allowPredict = false;
    end

    [theta, thetaDot, thetaDDot, theta3, theta4] = ...
        trajectoryTheta(t, cfg, allowPredict);

    [x, vx, ax, jx, sx] = trigDerivatives( ...
        cfg.Ax, theta, thetaDot, thetaDDot, theta3, theta4, "cos");
    [y, vy, ay, jy, sy] = trigDerivatives( ...
        cfg.Ay, theta, thetaDot, thetaDDot, theta3, theta4, "sin");

    ref.p = [x; y; -cfg.Hc];
    ref.v = [vx; vy; 0];
    ref.a = [ax; ay; 0];
    ref.j = [jx; jy; 0];
    ref.s = [sx; sy; 0];
    ref = setHeadingFromVelocity(ref, 0);
end

function fBody = sunAeroForceBody(R, v, par)

    if ~isfield(par, 'aero') || ~par.aero.enabled
        fBody = zeros(3,1);
        return;
    end

    vBody = R'*v;
    kd = par.aero.kd(:);
    kh = par.aero.kh;

    % Sun Eq. (9), transformed to this benchmark's FRD body frame. The paper's
    % +k_h term is along the thrust-aligned body z; here body +z is opposite
    % thrust, so the induced-drag term enters with a minus sign.
    lateralSpeedSq = vBody(1)^2 + vBody(2)^2;
    fBody = [-kd(1)*vBody(1);
             -kd(2)*vBody(2);
             -kd(3)*vBody(3) - kh*lateralSpeedSq];
end

%% Controller layer
function u = controller(x, ref, traj, t, par)

    switch par.controllerName
        case "geometric"
            u = scheduledSample("geometric_direct", t, ...
                par.control.innerPeriod, par, ...
                @() controllerPDGeometric(x, ref, par));
        case "lee"
            u = scheduledSample("lee_direct", t, ...
                par.control.innerPeriod, par, ...
                @() controllerLee(x, ref, par));
        case "px4_iris"
            u = controllerPX4Iris(x, ref, t, par);
        case "johnson"
            u = scheduledSample("johnson_direct", t, ...
                par.control.innerPeriod, par, ...
                @() controllerJohnson(x, ref, t, par));
        case "sun_nmpc"
            u = controllerSunNMPC(x, ref, traj, t, par);
        case "sun_dfbc"
            u = scheduledSample("sun_dfbc_direct", t, ...
                par.control.innerPeriod, par, ...
                @() controllerSunDFBC(x, ref, traj, t, par));
        case "sun_nmpc_indi"
            u = controllerSunNMPCINDI(x, ref, traj, t, par);
        case "sun_dfbc_indi"
            u = controllerSunDFBCINDI(x, ref, traj, t, par);
        case "lu"
            u = controllerLu(x, ref, traj, t, par);
        case "geometric_indi"
            u = scheduledSample("geometric_indi_inner", t, ...
                par.control.innerPeriod, par, ...
                @() controllerGeometricINDI(x, ref, t, par));
        case "tal"
            u = scheduledSample("tal_inner", t, ...
                par.control.innerPeriod, par, ...
                @() controllerTal(x, ref, traj, t, par));
        otherwise
            error("Unknown controllerName.");
    end
end

function value = scheduledSample(key, t, period, par, computeFn)

    persistent cache

    if isempty(cache)
        cache = struct();
    end

    field = matlab.lang.makeValidName(char(key));
    resetState = ~isfield(cache, field) || t <= par.dt/2 || t <= cache.(field).t;
    if resetState
        cache.(field).nextTime = t;
    end

    sampleDue = resetState || t + 0.5*par.dt >= cache.(field).nextTime;
    if sampleDue
        value = computeFn();
        cache.(field).value = value;
        cache.(field).t = t;
        cache.(field).nextTime = advanceSampleTime( ...
            cache.(field).nextTime, t, period, par.dt);
    else
        value = cache.(field).value;
    end
end

function nextTime = advanceSampleTime(nextTime, t, period, baseDt)

    nextTime = nextTime + period;
    while nextTime <= t + 0.5*baseDt
        nextTime = nextTime + period;
    end
end

function u = controllerPDGeometric(x, ref, par)

    % Algorithm model/control law:
    %   p_dot = v
    %   v_dot = g*e3 - c*b3, where b3 = R*e3 and c = T/m
    %   R_dot = R*hat(Omega)
    %   J*Omega_dot = tau - Omega x J*Omega
    %   aCmd = a_d + Kp*(p_d-p) + Kv*(v_d-v)
    %   c = ||g*e3-aCmd||, b3d = (g*e3-aCmd)/c
    %   OmegaDotCmd = KR*Log(R'*Rd) + KOmega*(R'*Rd*OmegaD-Omega)
    %                 + alphaD expressed in the body frame
    %   tau = Omega x J*Omega + J*OmegaDotCmd.
    ref = completeReferenceDerivatives(ref);

    % Position errors: ep = p_d-p, ev = v_d-v.
    ep = ref.p - x.p;
    ev = ref.v - x.v;
    % GINDI Eq. (56), non-INDI form: aFb = Kp*ep + Kv*ev.
    aFb = par.geometric.Kp*ep + par.geometric.Kv*ev;
    % Translational command: a_c = a_d + a_fb.
    aCmd = ref.a + aFb;

    % Direct non-INDI inversion of Eq. (53):
    %   c*b3d = g*e3 - a_c,  T = m*c.
    bodyZAccel = par.g*par.e3 - aCmd;
    c = norm(bodyZAccel);
    T = par.m*c;

    % Desired attitude: b3d = bodyZAccel/||bodyZAccel||, then impose yaw.
    b3d = bodyZAccel/c;
    [xC, xCDot, xCDDot] = headingAxisFromYaw(ref);
    Rd = attitudeFromUnitBodyZAndHeading(b3d, xC);

    % Desired attitude derivatives: differentiate the same body +z_B
    % vector and yaw-heading chain, then map Rd_dot/Rd_ddot to OmegaD.
    b3 = x.R*par.e3;
    b3Dot = x.R*hat(x.Omega)*par.e3;
    % Closed-loop acceleration used for analytic derivatives:
    %   a = g*e3 - c*b3.
    aActual = par.g*par.e3 - c*b3;
    % d(g*e3-a_c)/dt = Kp*(v-v_d) + Kv*(a-a_d) - j_d.
    bodyZAccelDot = par.geometric.Kp*(x.v - ref.v) ...
        + par.geometric.Kv*(aActual - ref.a) - ref.j;
    % cDot = b3d' * bodyZAccelDot for c = ||bodyZAccel||.
    cDot = dot(b3d, bodyZAccelDot);
    % aDot = d(g*e3-c*b3)/dt = -(cDot*b3 + c*b3Dot).
    aActualDot = -cDot*b3 - c*b3Dot;
    % d2(g*e3-a_c)/dt2 = Kp*(a-a_d) + Kv*(aDot-j_d) - s_d.
    bodyZAccelDDot = par.geometric.Kp*(aActual - ref.a) ...
        + par.geometric.Kv*(aActualDot - ref.j) - ref.s;

    [b3d, b3dDot, b3dDDot] = unitVectorDerivativesFromVector( ...
        bodyZAccel, bodyZAccelDot, bodyZAccelDDot);
    [RdDot, RdDDot] = attitudeDerivativesFromUnitBodyZAndHeading( ...
        b3d, b3dDot, b3dDDot, xC, xCDot, xCDDot);
    OmegaD = vee(Rd' * RdDot);
    alphaD = vee(Rd' * RdDDot - hat(OmegaD)*hat(OmegaD));

    Rbd = x.R' * Rd;

    % SO(3) attitude errors:
    %   rErr = Log(R'*Rd),
    %   omegaErr = R'*Rd*OmegaD - Omega.
    rErr = LogSO3(Rbd);
    omegaDInBody = Rbd * OmegaD;
    omegaErr = omegaDInBody - x.Omega;
    % alphaDInBody = R'*Rd*alphaD - Omega x (R'*Rd*OmegaD).
    alphaDInBody = Rbd * alphaD - hat(x.Omega)*omegaDInBody;
    % Non-INDI counterpart of GINDI Eq. (61):
    %   OmegaDotCmd = KR*rErr + KOmega*omegaErr + alphaDInBody.
    omegaDotCmd = par.geometric.KR*rErr ...
                + par.geometric.KOmega*omegaErr ...
                + alphaDInBody;
    % Rigid-body inversion:
    %   tau = Omega x J*Omega + J*OmegaDotCmd.
    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.J*omegaDotCmd;

    u = controlAllocation([T; tau], Rd, par);
    u.OmegaD = omegaDInBody;
    u.alphaD = alphaDInBody;
end

function u = controllerLee(x, ref, par)

    % Algorithm model/control law:
    %   p_dot = v
    %   v_dot = g*e3 - T/m*R*e3
    %   R_dot = R*hat(Omega)
    %   J*Omega_dot = tau - Omega x J*Omega
    % Lee Eq. (19)-(23), rewritten for this NED/FRD plant:
    %   F = Kx*(p-p_d) + Kv*(v-v_d) + m*(g*e3-a_d)
    %   b3c = F/||F||, T = F'*(R*e3)
    %   eR = 0.5*vee(Rc'*R - R'*Rc)
    %   eOmega = Omega - R'*Rc*OmegaC
    %   tau = -KR*eR - KOmega*eOmega + Omega x J*Omega
    %         - J*(hat(Omega)*R'*Rc*OmegaC - R'*Rc*OmegaCDot).
    ref = completeReferenceDerivatives(ref);

    % Lee position errors: ex = p-p_d, ev = v-v_d.
    ex = x.p - ref.p;
    ev = x.v - ref.v;

    % Position controller output:
    %   F = Kp*ex + Kv*ev + m*(g*e3-a_d).
    bodyZVector = par.lee.Kp*ex + par.lee.Kv*ev ...
        + par.m*(par.g*par.e3 - ref.a);
    % Lee thrust projection: T = F' * b3.
    T = dot(bodyZVector, x.R*par.e3);

    % Desired attitude: b3c = F/||F||, then impose yaw.
    b3d = bodyZVector/norm(bodyZVector);
    [xC, xCDot, xCDDot] = headingAxisFromYaw(ref);
    Rc = attitudeFromUnitBodyZAndHeading(b3d, xC);

    % 3. Desired attitude derivatives: differentiate the same body +z_B
    % vector and yaw-heading chain, then map Rc_dot/Rc_ddot to Omega_c.
    % Lee Appendix F differentiates A with b3c=-A/||A||. In this NED/FRD
    % implementation the attitude block receives F=-A=T*b3c, not the
    % physical thrust force -T*b3c.
    b3 = x.R*par.e3;
    b3Dot = x.R*hat(x.Omega)*par.e3;
    % Closed-loop acceleration under Lee's projected thrust:
    %   a = g*e3 - T/m*b3.
    aActual = par.g*par.e3 - T/par.m*b3;
    % FDot = Kp*ev + Kv*(a-a_d) - m*j_d.
    bodyZVectorDot = par.lee.Kp*ev ...
        + par.lee.Kv*(aActual - ref.a) - par.m*ref.j;
    % Since T = F'*b3, TDot = FDot'*b3 + F'*b3Dot.
    TDot = dot(bodyZVectorDot, b3) + dot(bodyZVector, b3Dot);
    % aDot = -TDot/m*b3 - T/m*b3Dot.
    aActualDot = -TDot/par.m*b3 - T/par.m*b3Dot;
    % FDDot = Kp*(a-a_d) + Kv*(aDot-j_d) - m*s_d.
    bodyZVectorDDot = par.lee.Kp*(aActual - ref.a) ...
        + par.lee.Kv*(aActualDot - ref.j) - par.m*ref.s;

    [b3d, b3dDot, b3dDDot] = unitVectorDerivativesFromVector( ...
        bodyZVector, bodyZVectorDot, bodyZVectorDDot);
    [RcDot, RcDDot] = attitudeDerivativesFromUnitBodyZAndHeading( ...
        b3d, b3dDot, b3dDDot, xC, xCDot, xCDDot);
    OmegaC = vee(Rc' * RcDot);
    OmegaCDot = vee(Rc' * RcDDot - hat(OmegaC)*hat(OmegaC));

    % Lee 2010 Eq. (97): OmegaC and OmegaCDot come from Appendix F's
    % analytic Rc_dot/Rc_ddot chain.
    % Lee's attitude error is proportional to sin(theta) for a relative
    % rotation angle theta. Near large attitude errors, especially close to
    % 180 deg, the error signal weakens even though the true SO(3) distance
    % is large. On aggressive helix_flip segments this is the primary cause
    % of loss of tracking; the thrust projection T = F'*R*e3 then amplifies
    % the attitude lag into translational error. Keep this unchanged for the
    % paper-faithful Lee controller.
    % Attitude errors: eR = 0.5*vee(Rc'*R - R'*Rc),
    % eOmega = Omega - R'*Rc*OmegaC.
    eR = 0.5*vee(Rc' * x.R - x.R' * Rc);
    eOmega = x.Omega - x.R' * Rc * OmegaC;

    % Lee et al. Eq. (12)-(13), rewritten for the NED plant.
    tau = -par.lee.KR*eR - par.lee.KOmega*eOmega ...
        + cross(x.Omega, par.J*x.Omega) ...
        - par.J*(hat(x.Omega)*x.R' * Rc * OmegaC - x.R' * Rc * OmegaCDot);

    u = controlAllocation([T; tau], Rc, par);
    u.OmegaD = OmegaC;
    u.alphaD = OmegaCDot;
end

function [xC, xCDot, xCDDot] = headingAxisFromYaw(ref)

    psi = ref.psi;
    psiDot = ref.psiDot;
    psiDDot = ref.psiDDot;

    xC = [cos(psi); sin(psi); 0];
    xCDot = psiDot*[-sin(psi); cos(psi); 0];
    xCDDot = psiDDot*[-sin(psi); cos(psi); 0] - psiDot^2*xC;
end

function Rd = attitudeFromUnitBodyZAndHeading(b3d, xC)

    C = cross(b3d, xC);
    b2d = C/norm(C);
    b1d = cross(b2d, b3d);
    Rd = [b1d, b2d, b3d];
end

function [RdDot, RdDDot] = attitudeDerivativesFromUnitBodyZAndHeading( ...
        b3d, b3dDot, b3dDDot, xC, xCDot, xCDDot)

    % Appendix F writes C=b3c x b1d. The printed b2c=-C/||C|| gives the
    % opposite yaw direction for this code's convention, so use b2=+C/||C||.
    C = cross(b3d, xC);
    CDot = cross(b3dDot, xC) + cross(b3d, xCDot);
    CDDot = cross(b3dDDot, xC) ...
        + 2*cross(b3dDot, xCDot) + cross(b3d, xCDDot);

    [b2d, b2dDot, b2dDDot] = unitVectorDerivativesFromVector( ...
        C, CDot, CDDot);

    b1dDot = cross(b2dDot, b3d) + cross(b2d, b3dDot);
    b1dDDot = cross(b2dDDot, b3d) ...
        + 2*cross(b2dDot, b3dDot) + cross(b2d, b3dDDot);

    RdDot = [b1dDot, b2dDot, b3dDot];
    RdDDot = [b1dDDot, b2dDDot, b3dDDot];
end

function [b, bDot, bDDot] = unitVectorDerivativesFromVector(v, vDot, vDDot)

    % Common formula for b=v/||v||. Callers explicitly construct v, vDot,
    % and vDDot from their own controller equations.
    %   rho = ||v||, P = I - b*b'
    %   bDot = P*vDot/rho
    %   bDDot = P*vDDot/rho
    %           - 2*(b'*vDot)/rho*bDot - ||bDot||^2*b.
    rho = norm(v);
    b = v/rho;
    P = eye(numel(v)) - b*b';

    bDot = P*vDot/rho;
    bDDot = P*vDDot/rho ...
        - 2*dot(b, vDot)/rho*bDot ...
        - dot(bDot, bDot)*b;
end

function u = controllerPX4Iris(x, ref, t, par)

    persistent st

    % Algorithm model/control law:
    %   This is a PX4-style cascade, expressed in normalized actuator units:
    %     position P -> velocity PID+D -> thrust vector thrSp,
    %     thrust vector + yaw -> Rd,
    %     quaternion attitude P -> body-rate setpoint,
    %     body-rate PID -> normalized torque.
    %   The plant still receives the benchmark allocation output, but
    %   controlAllocationPX4Normalized interprets
    %     muNormCmd = [-||thrSp||; torqueNorm]
    %   with PX4 hover-thrust scaling and normalized moment limits.
    p = par.px4_iris;
    if isempty(st) || t <= par.dt/2 || t <= st.t
        st.velInt = zeros(3,1);
        st.prevVelOuter = x.v;
        st.rateInt = zeros(3,1);
        st.prevOmegaRate = x.Omega;
        st.thrSp = [0; 0; -p.hoverThrust];
        st.Rd = x.R;
        st.velSp = x.v;
        st.accSp = zeros(3,1);
        st.ratesSp = zeros(3,1);
        st.torqueNorm = zeros(3,1);
        st.outerT = t - par.control.innerPeriod;
        st.rateT = t - par.control.innerPeriod;
        st.nextOuterTime = t;
        st.nextRateTime = t;
        st.t = t - par.dt;
    end

    outerDue = t + 0.5*par.dt >= st.nextOuterTime;
    if outerDue
        hOuter = max(t - st.outerT, par.control.innerPeriod);
        % Measured acceleration estimate for the D term:
        %   velDot = d(v)/dt.
        velDot = (x.v - st.prevVelOuter)/max(hOuter, eps);

        % PX4 position/velocity loop:
        %   velSp, accSp, thrSp, Rd.
        [thrSp, Rd, st] = px4PositionControl(x, ref, velDot, par, st, hOuter);

        % PX4 attitude P loop:
        %   ratesSp = 2*sign(qe_w)*qe_xyz .* attP.
        ratesSp = px4AttitudeControl(x.R, Rd, p);
        if getStructField(p, 'useYawRateFeedforward', true)
            % PX4 mc_att_control adds only the input yaw_sp_move_rate
            % feed-forward in the current body frame, not a full flatness
            % angular-rate reference generated from Rd(t).
            ratesSp = ratesSp + x.R' * [0; 0; 1] * ref.psiDot;
        end

        st.thrSp = thrSp;
        st.Rd = Rd;
        st.ratesSp = ratesSp;
        st.prevVelOuter = x.v;
        st.outerT = t;
        st.nextOuterTime = advanceSampleTime( ...
            st.nextOuterTime, t, par.control.innerPeriod, par.dt);
    end

    rateDue = t + 0.5*par.dt >= st.nextRateTime;
    if rateDue
        hRate = max(t - st.rateT, par.control.innerPeriod);
        % Body-rate derivative estimate for the PX4 D term.
        angularAccel = (x.Omega - st.prevOmegaRate)/max(hRate, eps);

        % PX4 body-rate PID:
        %   torqueNorm = P*(ratesSp-rates) + I - D*OmegaDot + FF*ratesSp.
        [torqueNorm, st] = px4RateControl( ...
            x.Omega, st.ratesSp, angularAccel, p, st, hRate);

        st.torqueNorm = torqueNorm;
        st.prevOmegaRate = x.Omega;
        st.rateT = t;
        st.nextRateTime = advanceSampleTime( ...
            st.nextRateTime, t, par.control.innerPeriod, par.dt);
    end

    thrustBodyZ = -norm(st.thrSp);
    muNormCmd = [thrustBodyZ; st.torqueNorm];
    u = controlAllocationPX4Normalized(muNormCmd, st.Rd, par);
    u.vPlotD = st.velSp;
    u.aPlotD = st.accSp;
    u.OmegaD = st.ratesSp;

    st.t = t;
end

function [thrSp, Rd, st] = px4PositionControl(x, ref, velDot, par, st, dt)

    p = par.px4_iris;

    % Position P + velocity feed-forward. Deliberately keep this benchmark
    % variant unconstrained; only the final motor commands are saturated.
    velSpPosition = (ref.p - x.p) .* p.posP;
    velSp = ref.v + velSpPosition;

    velError = velSp - x.v;

    if p.useAccelerationFeedforward
        accFF = ref.a;
    else
        accFF = zeros(3,1);
    end

    accSp = accFF + velError .* p.velP + st.velInt - velDot .* p.velD;
    st.velSp = velSp;
    st.accSp = accSp;

    % Acceleration-control map: accSp -> normalized thrust vector.
    thrSp = px4AccelerationControl(accSp, p, par);

    st.velInt = st.velInt + velError .* p.velI * dt;

    % Attitude setpoint from thrust direction and yaw.
    bodyZ = -thrSp;
    Rd = px4BodyZToAttitude(bodyZ, ref.psi);
end

function thrSp = px4AccelerationControl(accSp, p, par)

    % PX4 specific-force direction. With MPC_ACC_DECOUPLE disabled, the
    % vertical acceleration feed-forward also changes the desired tilt.
    zSpecificForce = -par.g;
    if ~getStructField(p, 'decoupleHorizontalAndVerticalAcceleration', true)
        zSpecificForce = zSpecificForce + accSp(3);
    end
    % PX4 specific-force direction. With MPC_ACC_DECOUPLE disabled, the
    % vertical acceleration feed-forward also changes the desired tilt.
    zSpecificForce = -par.g;
    if ~getStructField(p, 'decoupleHorizontalAndVerticalAcceleration', true)
        zSpecificForce = zSpecificForce + accSp(3);
    end
    bodyZ = [-accSp(1); -accSp(2); -zSpecificForce];
    bodyZ = bodyZ/norm(bodyZ);

    % Normalized collective thrust with vertical acceleration compensation:
    %   thrustNedZ = accSp_z*hoverThrust/g - hoverThrust
    %   collectiveThrust = thrustNedZ/(e3'*bodyZ).
    thrustNedZ = accSp(3) * (p.hoverThrust/par.g) - p.hoverThrust;
    cosNedBody = dot(par.e3, bodyZ);
    collectiveThrust = thrustNedZ/cosNedBody;
    thrSp = bodyZ * collectiveThrust;
end

function Rd = px4BodyZToAttitude(bodyZ, yawSp)

    % PX4 attitude construction:
    %   bodyZ = normalize(bodyZ),
    %   yC = [-sin(psi); cos(psi); 0],
    %   bodyX = normalize(yC x bodyZ),
    %   bodyY = bodyZ x bodyX.
    bodyZ = bodyZ/norm(bodyZ);

    % PX4 constrains the yaw heading through y_C instead of Lee/JB's b1 axis.
    % With psi=0 this singular direction is the y axis, so this construction
    % is suited to helix_flip_y rather than the x-forward helix_flip case.
    yC = [-sin(yawSp); cos(yawSp); 0];
    bodyX = cross(yC, bodyZ);
    bodyX = bodyX/norm(bodyX);

    bodyY = cross(bodyZ, bodyX);
    Rd = [bodyX, bodyY, bodyZ];
end

function ratesSp = px4AttitudeControl(R, Rd, p)

    % Quaternion attitude error:
    %   qe = q(R)^(-1) * q(Rd), qe_w >= 0.
    % q = rotm2quat(R).';
    % qd = rotm2quat(Rd).';
    % qError = quatnormalize(quatmultiply([q(1), -q(2:4).'], qd.')).';
    % if qError(1) < 0
    %     qError = -qError;
    % end
    % q = rotm2quat(R).';
    % qd = rotm2quat(Rd).';
    % qError = quatnormalize(quatmultiply([q(1), -q(2:4).'], qd.')).';
    % if qError(1) < 0
    %     qError = -qError;
    % end

    % PX4 attitude P law:
    %   ratesSp = 2*qe_xyz .* attP, then apply rate limits.
    % ratesSp = 2*qError(2:4) .* p.attP;
    R_e=R'*Rd;
    ratesSp = p.attP .* LogSO3(R_e);
    % ratesSp = 2*qError(2:4) .* p.attP;
    R_e=R'*Rd;
    ratesSp = p.attP .* LogSO3(R_e);
    ratesSp = saturateVector(ratesSp, getStructField(p, 'rateLimit', inf));
end

function [torqueNorm, st] = px4RateControl( ...
        rates, ratesSp, angularAccel, p, st, dt)

    % Rate error:
    %   eRate = ratesSp - rates.
    rateError = ratesSp - rates;
    % PX4 rate PID in normalized torque units:
    %   torqueNorm = P.*eRate + I - D.*OmegaDot + FF.*ratesSp.
    torqueNorm = p.rateP .* rateError + st.rateInt ...
        - p.rateD .* angularAccel + p.rateFF .* ratesSp;

    for i = 1:3
        % PX4 integrator attenuation near large rate errors, then clamp.
        iFactor = rateError(i)/deg2rad(400);
        iFactor = max(0, 1 - iFactor^2);
        nextInt = st.rateInt(i) + iFactor*p.rateI(i)*rateError(i)*dt;
        st.rateInt(i) = min(max(nextInt, -p.rateIntLimit(i)), ...
            p.rateIntLimit(i));
    end
end

function u = controllerJohnson(x, ref, t, par)

    persistent st

    % Algorithm model/control law:
    %   p_dot = v
    %   v_dot = g*e3 - T/m*R*e3
    %   R_dot = R*hat(Omega)
    %   J*Omega_dot = tau - Omega x J*Omega
    % Johnson and Beard Eq. (13), (18)-(21):
    %   ea = [p-p_d; v-v_d; integral(p-p_d)]
    %   fd = -K*ea + m*(a_d-g*e3)
    %   k_d = -fd/||fd||, T = ||fd||
    %   r = Log(R'*Rid)
    %   tau = Omega x J*Omega + J*omegaDotD_B
    %         + Jl(r)'*Kr*r + Komega*(R'*Rid*omegaD-Omega).
    ref = completeReferenceDerivatives(ref);

    % Johnson translational errors: ep = p-p_d, ev = v-v_d.
    ep = x.p - ref.p;
    ev = x.v - ref.v;

    if isempty(st) || t <= par.dt/2 || t <= st.t
        st.eInt = zeros(3,1);
        st.t = t - par.dt;
    end

    h = max(t - st.t, par.dt);
    % Integral error state: eI_dot = ep.
    st.eInt = st.eInt + h*ep;

    % Augmented LQR state: ea = [ep; ev; eI].
    ea = [ep; ev; st.eInt];
    K = johnsonPositionGain(par);

    % Desired force:
    %   fEq = m*(a_d-g*e3), fTilde = -K*ea, fd = fTilde + fEq.
    fEq = par.m*(ref.a - par.g*par.e3);
    fTilde = -K*ea;
    fd = fTilde + fEq;
    % Johnson Eq. (19a): body +z_B axis uses k_d = -fd/||fd||.
    bodyZVector = -fd;
    T = norm(fd);

    % Desired attitude: b3d = -fd/||fd||, then impose yaw.
    b3d = bodyZVector/norm(bodyZVector);
    [xC, xCDot, xCDDot] = headingAxisFromYaw(ref);
    Rid = attitudeFromUnitBodyZAndHeading(b3d, xC);

    % 3. Desired attitude derivatives: differentiate the same body +z_B
    % vector and yaw-heading chain, then map Rid_dot/Rid_ddot to omega_d.
    b3 = x.R*par.e3;
    b3Dot = x.R*hat(x.Omega)*par.e3;
    % Closed-loop acceleration:
    %   a = g*e3 - T/m*b3.
    aActual = par.g*par.e3 - T/par.m*b3;
    % eaDot = [ev; a-a_d; ep].
    eaDot = [ev; aActual - ref.a; ep];
    % fdDot = -K*eaDot + m*j_d.
    fdDot = -K*eaDot + par.m*ref.j;
    bodyZVectorDot = -fdDot;
    % T = ||fd|| = ||-bodyZVector||, so TDot = b3d'*bodyZVectorDot.
    TDot = dot(b3d, bodyZVectorDot);
    % aDot = -TDot/m*b3 - T/m*b3Dot.
    aActualDot = -TDot/par.m*b3 - T/par.m*b3Dot;
    % eaDDot = [a-a_d; aDot-j_d; ev].
    eaDDot = [aActual - ref.a; aActualDot - ref.j; ev];
    % fdDDot = -K*eaDDot + m*s_d.
    fdDDot = -K*eaDDot + par.m*ref.s;
    bodyZVectorDDot = -fdDDot;

    [b3d, b3dDot, b3dDDot] = unitVectorDerivativesFromVector( ...
        bodyZVector, bodyZVectorDot, bodyZVectorDDot);
    [RidDot, RidDDot] = attitudeDerivativesFromUnitBodyZAndHeading( ...
        b3d, b3dDot, b3dDDot, xC, xCDot, xCDDot);
    omegaD = vee(Rid' * RidDot);
    omegaDotD = vee(Rid' * RidDDot - hat(omegaD)*hat(omegaD));

    Rbd = x.R' * Rid;
    % SO(3) errors: r = Log(R'*Rid),
    % omegaErr = R'*Rid*omegaD - Omega.
    r = johnsonLogSO3(Rbd);
    omegaDInBody = Rbd * omegaD;
    omegaErr = omegaDInBody - x.Omega;
    % Body-frame reference angular acceleration:
    % omegaDotD_B = R'*Rid*omegaDotD - Omega x (R'*Rid*omegaD).
    omegaDotDInBody = Rbd * omegaDotD - hat(x.Omega)*omegaDInBody;

    Jl = johnsonLeftJacobianSO3(r);
    % Johnson and Beard Eq. (21)-(23), (29)-(32) torque law.
    tau = cross(x.Omega, par.J*x.Omega) ...
        + par.J*omegaDotDInBody ...
        + Jl' * par.johnson.Kr*r ...
        + par.johnson.Komega*omegaErr;

    u = controlAllocation([T; tau], Rid, par);
    u.OmegaD = omegaDInBody;
    u.alphaD = omegaDotDInBody;
    st.t = t;
end

function K = johnsonPositionGain(par)

    mode = lower(string(getStructField(par.johnson, ...
        'positionGainMode', "lqr")));

    switch mode
        case "lqr"
            K = par.johnson.Klqr;
        case "pd"
            Kp = getStructField(par.johnson, 'Kp', par.lee.Kp);
            Kv = getStructField(par.johnson, 'Kv', par.lee.Kv);
            Ki = getStructField(par.johnson, 'Ki', zeros(3));
            K = [Kp, Kv, Ki];
        otherwise
            error('Unknown par.johnson.positionGainMode "%s".', mode);
    end
end

function phiVec = johnsonLogSO3(R)

    % Johnson and Beard Eq. (5a)-(5b).
    phi = acos((trace(R) - 1)/2);

    if abs(abs(phi) - pi) < 1e-10
        [V, D] = eig(R);
        [~, i] = min(abs(diag(D) - 1));
        u = real(V(:,i));
        u = u/norm(u);
        phiVec = phi*u;
        return;
    end

    phiVec = 1/(2*johnsonSinc(phi/2)*cos(phi/2)) ...
        * vee(R - R');
end

function Jl = johnsonLeftJacobianSO3(phiVec)

    % Johnson and Beard Eq. (6).
    phi = norm(phiVec);

    if phi == 0
        Jl = eye(3);
        return;
    end

    uHat = hat(phiVec/phi);
    Jl = eye(3) ...
        + sin(phi/2)*johnsonSinc(phi/2)*uHat ...
        + (1 - johnsonSinc(phi))*uHat*uHat;
end

function y = johnsonSinc(x)

    if x == 0
        y = 1;
    else
        y = sin(x)/x;
    end
end

function ff = talFlatnessReference(ref, par)

    % Tal Eq. (8)-(13): flat outputs are x_ref and psi_ref. In this benchmark
    % v_dot = g*e3 - T/m*b_z, while Tal writes v_dot = g*i_z + tau*b_z with
    % signed specific thrust tau. Therefore tau_ref*b_z,ref = a_ref - g*e3.
    ref = completeReferenceDerivatives(ref);

    % Specific-thrust vector in Tal's sign convention:
    %   tau_ref*b3_ref = a_ref - g*e3.
    tauVector = ref.a - par.g*par.e3;
    tauMag = norm(tauVector);
    % Benchmark body +z axis is opposite the thrust acceleration direction:
    %   b3_ref = -(a_ref-g*e3)/||a_ref-g*e3||.
    b3 = -tauVector/tauMag;

    % Yaw completion:
    %   b2 = normalize(b3 x b1Yaw), b1 = b2 x b3.
    b1Yaw = [cos(ref.psi); sin(ref.psi); 0];
    b2 = cross(b3, b1Yaw);
    b2 = b2/norm(b2);
    b1 = cross(b2, b3);

    ff.R = [b1, b2, b3];
    ff.tau = -tauMag;
    ff.c = tauMag;
    ff.T = par.m*tauMag;
end

function u = controllerTal(x, ref, ~, t, par)

    persistent st

    % Algorithm model/control law:
    %   Paper model: v_dot = g*i_z + tau*b_z + f_ext/m, tau < 0 for lift.
    %   Benchmark model: v_dot = g*e3 - T/m*R*e3, so tau = -T/m.
    %   aCmd = Kp*(p_d-p) + Kv*(v_d-v) + Ka*(a_d-a_f) + a_d
    %   (tau*b_z)_c = (tau*b_z)_f + aCmd - a_f
    %   xi_e comes from the incremental attitude command.
    %   OmegaDotCmd = Ktheta*xi_e + Komega*(Omega_ref-Omega_f)
    %                 + alpha_ref
    %   tauCmd = mu_f + J*(OmegaDotCmd-OmegaDot_f).
    % Tal and Karaman 2021, adapted to this MATLAB benchmark:
    % - Paper model uses NED and v_dot = g*i_z + tau*b_z + f_ext/m, where
    %   tau is the signed specific thrust. This benchmark instead outputs a
    %   positive force T and uses v_dot = g*e3 - T/m*b_z, so tau = -T/m.
    % - Paper Eq. (16), (17), (20), (28), and (31) are kept. In simulation,
    %   a_f and omegaDot_f are evaluated from the plant model under the
    %   previously applied saturated command, then filtered with the Fig. 4
    %   second-order Butterworth LPF. Eq. (20)'s filtered specific-thrust
    %   vector is represented by the previous saturated force command st.T,
    %   converted from N to m/s^2 in the current attitude and passed through
    %   the same LPF. Eq. (31)'s mu_f is handled similarly for the direct
    %   equivalent moment.
    % - Paper Eq. (22)-(26) builds an incremental quaternion attitude command.
    %   This framework stores absolute attitude commands, so we compute the
    %   equivalent Rd from the INDI thrust vector and yaw; x.R'*Rd is the
    %   incremental attitude used in Eq. (28).
    % - Paper Eq. (32)-(36) need rotor-speed dynamics and ESC throttle states,
    %   which this benchmark does not have. Allocation is intentionally shared
    %   with the other controllers; Eq. (31)'s filtered moment mu_f is
    %   represented by the previous allocated equivalent moment st.tau.

    ff = talFlatnessReference(ref, par);

    filterReset = isempty(st) || t <= par.dt/2 || t <= st.t;
    if filterReset
        aFilt = ref.a;
        omegaF = x.Omega;
        omegaDotF = zeros(3,1);
        thrustAccelF = ff.tau * ff.R*par.e3;
        muF = zeros(3,1);

        st.aFilter = initSecondOrderLPF(aFilt);
        st.omegaFilter = initSecondOrderLPF(omegaF);
        st.omegaDotFilter = initSecondOrderLPF(omegaDotF);
        st.thrustAccelFilter = initSecondOrderLPF(thrustAccelF);
        st.muFilter = initSecondOrderLPF(muF);
    else
        h = max(t - st.t, par.dt);
        rawOmegaF = x.Omega;
        [rawAFilt, rawOmegaDotF] = rigidBodyRates( ...
            x.R, x.v, x.Omega, st.uApplied, par, t);
        % Previous applied force T [N] in current attitude:
        %   (tau*b_z)_f = -T_prev/m * R*e3.
        rawThrustAccelF = -st.T/par.m * x.R*par.e3;
        rawMuF = st.tau;

        [aFilt, st.aFilter] = secondOrderButterworthLPF( ...
            rawAFilt, st.aFilter, h, par.tal.outerLoopFilterCutoffHz);
        [omegaF, st.omegaFilter] = secondOrderButterworthLPF( ...
            rawOmegaF, st.omegaFilter, h, par.tal.innerLoopFilterCutoffHz);
        [omegaDotF, st.omegaDotFilter] = secondOrderButterworthLPF( ...
            rawOmegaDotF, st.omegaDotFilter, h, par.tal.innerLoopFilterCutoffHz);
        [thrustAccelF, st.thrustAccelFilter] = secondOrderButterworthLPF( ...
            rawThrustAccelF, st.thrustAccelFilter, h, par.tal.outerLoopFilterCutoffHz);
        [muF, st.muFilter] = secondOrderButterworthLPF( ...
            rawMuF, st.muFilter, h, par.tal.innerLoopFilterCutoffHz);
    end

    % Eq. (17): commanded acceleration with acceleration feedback:
    %   aCmd = Kp*(p_d-p) + Kv*(v_d-v) + Ka*(a_d-a_f) + a_d.
    aCmd = par.tal.Kp*(ref.p - x.p) ...
         + par.tal.Kv*(ref.v - x.v) ...
         + par.tal.Ka*(ref.a - aFilt) ...
         + ref.a;

    % Eq. (20): INDI linear acceleration control. thrustAccelCmd is the
    % paper's vector (tau*b_z)_c. In this NED framework tau = -T/m, so the
    % thrust acceleration vector is -T/m*b_z:
    %   thrustAccelCmd = thrustAccelF + aCmd - aFilt.
    thrustAccelCmd = thrustAccelF + aCmd - aFilt;
    [Rd, xiE, T] = talIncrementalAttitudeCommand(x.R, thrustAccelCmd, ref.psi, par);

    % Eq. (14)-(15): solve Tal's 4x4 flatness matrix directly from jerk,
    % snap, yaw rate, and yaw acceleration. No SO(3) finite-difference
    % replacement is used when Eq. (11)-(13)'s yaw parametrization is
    % singular or ill-conditioned.
    refDer = completeReferenceDerivatives(ref);
    [omegaRef, alphaRef] = talPaperFlatnessFeedforward(ff.R, ff.tau, refDer, par);

    % Eq. (28): attitude/rate controller:
    %   omegaDotCmd = Ktheta*xiE + Komega*(omegaRef-omegaF) + alphaRef.
    omegaDotCmd = par.tal.Ktheta*xiE ...
                + par.tal.Komega*(omegaRef - omegaF) ...
                + alphaRef;

    % Eq. (31): INDI angular acceleration control. The paper uses filtered
    % motor-speed-derived moment mu_f; with direct moment actuation, muF is
    % the LPF output of previous saturated equivalent moments [N*m]:
    %   tau = muF + J*(omegaDotCmd-omegaDotF).
    tau = muF + par.J*(omegaDotCmd - omegaDotF);

    % Control allocation is benchmark-unified across controllers. Tal Eq.
    % (34)-(36) motor-speed inversion is not reproduced in this plant because
    % the simulator has no rotor-speed or ESC-throttle state.
    u = controlAllocation([T; tau], Rd, par);

    st.v = x.v;
    st.R = x.R;
    st.Omega = x.Omega;
    st.T = u.T;
    st.tau = u.tau;
    st.uApplied = u;
    u.OmegaD = omegaRef;
    u.alphaD = alphaRef;

    st.omegaDotF = omegaDotF;
    st.omegaF = omegaF;
    st.aFilt = aFilt;
    st.thrustAccelF = thrustAccelF;
    st.muF = muF;
    st.Rd = Rd;
    st.t = t;
end

function [Rd, xiE, T] = talIncrementalAttitudeCommand(R, thrustAccelCmd, psiRef, par)

    % Tal Eq. (21)-(27). thrustAccelCmd is (tau*b_z)_c in inertial NED.
    % Since tau is negative for upward thrust, the commanded body z axis is
    % b_z,c = -normalize((tau*b_z)_c).
    % Eq. (22): express commanded thrust acceleration in the current body
    % frame before constructing the incremental tilt quaternion.
    thrustDir = thrustAccelCmd/norm(thrustAccelCmd);
    thrustDirBody = R' * thrustDir;       % Eq. (22): (-b_z)_c in body frame.

    qTilt = talQuatAlignMinusE3(thrustDirBody);
    RTilt = quat2rotm(qTilt.');
    RIntermediate = R * RTilt;

    nPsi = [sin(psiRef); -cos(psiRef); 0];
    nBody = RIntermediate' * nPsi;        % Eq. (24).

    if nBody(2) == 0
        qYaw = [1; 0; 0; 0];
    else
        k = -nBody(1)/nBody(2);
        qYaw = [1; 0; 0; k]; % Correct the original text of tal
        qYaw = quatnormalize(qYaw.').';
    end

    qCmd = quatmultiply(qTilt.', qYaw.').'; % Eq. (26): current to command.
    qCmd = quatnormalize(qCmd.').';
    RCmd = quat2rotm(qCmd.');

    Rd = R * RCmd;
    xiE = talQuatErrorVector(qCmd);       % Eq. (27).
    % Eq. (21): paper uses signed T_c = -m*||(tau*b_z)_c||. The benchmark
    % actuator interface expects positive force magnitude, so only the sign
    % convention is converted here.
    T = par.m*norm(thrustAccelCmd);
end

function q = talQuatAlignMinusE3(vBody)

    vBody = vBody/norm(vBody);
    e3 = [0; 0; 1];
    c = dot(e3, vBody);
    axis = -cross(e3, vBody);

    % Tal Eq. (23) aligns current -b_z with (tau*b_z)_c. It is singular when
    % i_z = (-b_z)_c^b, i.e. a 180 deg tilt with arbitrary rotation axis.
    % The paper explicitly resolves this by selecting any direction of
    % rotation. Use a fixed body-x axis near that singularity. If
    % vBody == -i_z, the required tilt is zero.
    if c == 1
        q = [0; 1; 0; 0];
    elseif c == -1
        q = [1; 0; 0; 0];
    else
        q = [1 - c; axis];
        q = quatnormalize(q.').';
    end
end

function xiE = talQuatErrorVector(q)

    % Tal Eq. (27) is the quaternion logarithm written as
    %   xi_e = 2*acos(q_w)/sqrt(1-q_w^2) * q_v.
    % Benchmark policy: keep paper formulas as written and do not repair an
    % algorithm's assumptions for comparison. This identity-quaternion branch
    % is the exception: it only evaluates Eq. (27)'s removable mathematical
    % limit at q=[1;0;0;0]. Without it, exact hover starts produce numerical
    % 0/0 even though the paper log map is zero there.
    if q(1) == 1
        xiE = zeros(3,1);
        return;
    end

    xiE = 2*acos(q(1))/sqrt(1 - q(1)^2) * q(2:4);
end

function [omegaRef, alphaRef] = talPaperFlatnessFeedforward(R, tauSpec, refDer, par)

    % Tal Eq. (14)-(15). The paper's 4x4 matrix solves for
    % [omega_ref; tau_dot] and [omega_dot_ref; tau_ddot] from reference
    % jerk, snap, yaw rate, and yaw acceleration, using Eq. (11)-(13)'s
    % yaw-rate row.
    A = talFlatnessMatrix(R, tauSpec);

    yJerk = [refDer.j; refDer.psiDot];
    solJerk = A\yJerk;
    omegaRef = solJerk(1:3);
    tauDotRef = solJerk(4);

    omegaHat = hat(omegaRef);
    knownSnap = R*(2*tauDotRef*omegaHat + tauSpec*omegaHat*omegaHat)*par.e3;
    sDotOmega = talYawSdotOmega(R, omegaRef);
    ySnap = [refDer.s - knownSnap;
             refDer.psiDDot - sDotOmega];

    solSnap = A\ySnap;
    alphaRef = solSnap(1:3);
end

function A = talFlatnessMatrix(R, tauSpec)

    % Tal Eq. (14), in the same column order as the paper:
    %   [omega_ref; tau_dot_ref] =
    %       [tau*R*hat(e3)'  b3; S  0] \ [j_ref; psi_dot_ref].
    % Since hat(e3)' = -hat(e3), the upper-left block is
    % -tau*R*hat(e3).
    b3 = R*[0; 0; 1];
    S = talYawSRow(R);
    A = [-tauSpec*R*hat([0; 0; 1]), b3;
          S,                         0];
end

function S = talYawSRow(R)

    % Tal Eq. (11)-(13): psi = atan2(b_x,y, b_x,x), psi_dot = S(R)*omega.
    % The denominator is intentionally not regularized; when the horizontal
    % projection of b_x vanishes, the paper's yaw parametrization is singular.
    bx = R(:,1);
    den = bx(1)^2 + bx(2)^2;

    S = zeros(1,3);
    for i = 1:3
        e = zeros(3,1);
        e(i) = 1;
        bxDot = R*hat(e)*[1; 0; 0];
        S(i) = (bx(1)*bxDot(2) - bx(2)*bxDot(1))/den;
    end
end

function y = talYawSdotOmega(R, omega)

    % Tal Eq. (15): psi_ddot = S(R)*omega_dot + S_dot(R,omega)*omega.
    % Differentiate Eq. (11)-(13) analytically along R_dot = R*hat(omega).
    bx = R(:,1);
    bxDot = R*hat(omega)*[1; 0; 0];
    den = bx(1)^2 + bx(2)^2;
    denDot = 2*bx(1)*bxDot(1) + 2*bx(2)*bxDot(2);

    Sdot = zeros(1,3);
    for i = 1:3
        e = zeros(3,1);
        e(i) = 1;
        bxOmegaBasis = R*hat(e)*[1; 0; 0];
        bxOmegaBasisDot = R*hat(omega)*hat(e)*[1; 0; 0];

        num = bx(1)*bxOmegaBasis(2) - bx(2)*bxOmegaBasis(1);
        numDot = bxDot(1)*bxOmegaBasis(2) ...
               + bx(1)*bxOmegaBasisDot(2) ...
               - bxDot(2)*bxOmegaBasis(1) ...
               - bx(2)*bxOmegaBasisDot(1);
        Sdot(i) = (numDot*den - num*denDot)/den^2;
    end

    y = Sdot*omega;
end

function u = controllerSunNMPC(x, ~, traj, t, par)

    persistent st

    % Algorithm model/control law:
    %   Sun NMPC solves a finite-horizon OCP over
    %     x = [xi; q; xidot; Omega^B],
    %     u = [u1; u2; u3; u4] rotor thrusts.
    %   Dynamics use
    %     xi_dot = xidot,
    %     xidot_dot = g*e3 - T/m*R*e3 + R*f_a^B/m,
    %     R_dot = R*hat(Omega),
    %     J*Omega_dot = tau - Omega x J*Omega,
    %   where [T;tau] = G1*u. The first optimized actuator sample is mapped
    %   back through the benchmark allocation/control object.
    % Sun NMPC state is x=[xi; q; xidot; Omega^B] in this implementation
    % (the paper writes the same components as [xi; xidot; q; Omega^B]).
    % The optimized input is rotor thrust u=[u1;u2;u3;u4], not [T;tau];
    % G1*u gives [T;tau] inside the nonlinear dynamics. Sun Fig. 3 keeps
    % NMPC as the high-level block feeding INDI, and Sec. V/VI run the NMPC
    % OCP at 100 Hz; NMPC+INDI adds the Eq. (32)-(35) angular-acceleration
    % INDI loop at par.sun.indiPeriod.
    % Solver details, constraints, horizon, and cost weights are configured
    % through par.sun and implemented in tools/sun_acados_nmpc.py.
    cfg = par.sun;
    N = cfg.N;
    h = cfg.dt;
    solvePeriod = cfg.solvePeriod;
    refs = sunNMPCBuildReferences(traj, t, N, h, par);
    x0 = sunNMPCStateVector(x);

    solverReset = isempty(st) || t <= par.dt/2 || t <= st.t;
    if solverReset
        st = struct;
        st.pySolver = sunEnsureAcadosPython(par);
        st.pySolver.reset_warm_start();
        st.nextSolveTime = -inf;
        st.lastExitflag = nan;
    end

    solveDue = ~isfield(st, 'lastActuator') ...
        || t + 0.5*par.dt >= st.nextSolveTime;
    if ~solveDue
        u = sunActuatorToControl(st.lastActuator, refs.R(:,:,1), par);
        u.OmegaD = refs.Omega(:,1);
        u.alphaD = refs.alpha(:,1);
        u.sunNMPCCached = true;
        u.sunNMPCSolved = false;
        u.sunNMPCStatusCode = 2;
        u.sunNMPCExitflag = st.lastExitflag;
        u.sunNMPCSolveTime = 0;
        return;
    end

    % OCP control law: solve for the first actuator vector u0 over the
    % horizon references [xi_r, q_r, v_r, Omega_r, u_r].
    pyResult = st.pySolver.solve( ...
        py.numpy.array(x0'), ...
        py.numpy.array(refs.p'), ...
        py.numpy.array(refs.q'), ...
        py.numpy.array(refs.v'), ...
        py.numpy.array(refs.Omega'), ...
        py.numpy.array(refs.u'));

    status = double(pyResult{'status'});
    actuator = double(pyResult{'u0'});
    actuator = actuator(:);
    solveTime = double(pyResult{'solve_time'});

    u = sunActuatorToControl(actuator, refs.R(:,:,1), par);
    u.OmegaD = refs.Omega(:,1);
    u.alphaD = refs.alpha(:,1);
    u.sunNMPCCached = false;
    u.sunNMPCSolved = true;
    u.sunNMPCStatusCode = status;
    u.sunNMPCExitflag = status;
    u.sunNMPCSolveTime = solveTime;

    if cfg.printSolverTiming
        fprintf('sun_nmpc t=%.3f solve=%.4fs status=%d\n', ...
            t, solveTime, status);
    end

    st.lastActuator = actuator;
    st.nextSolveTime = t + solvePeriod;
    st.t = t;
    st.lastExitflag = status;
end

function pySolver = sunEnsureAcadosPython(par)

    persistent pyModule

    if isempty(pyModule)
        sunConfigurePythonForAcados(par);
        sunAssertAcadosInstallReady(par);

        setenv("ACADOS_SOURCE_DIR", char(par.sun.acadosSourceDir));
        setenv("ACADOS_INSTALL_DIR", char(par.sun.acadosSourceDir));
        setenv("MPLCONFIGDIR", char(fullfile(tempdir, "matplotlib")));

        toolsPath = char(par.sun.acadosToolsPath);
        if exist(toolsPath, 'dir') ~= 7
            mainPath = which('main');
            if strlength(mainPath) > 0
                toolsPath = fullfile(fileparts(mainPath), 'tools');
            end
        end

        pyPath = py.sys.path;
        pyPath.insert(int32(0), toolsPath);
        acadosTemplatePath = fullfile(char(par.sun.acadosSourceDir), ...
            "interfaces", "acados_template");
        if exist(acadosTemplatePath, 'dir') == 7
            pyPath.insert(int32(0), char(acadosTemplatePath));
        end
        try
            py.importlib.invalidate_caches();
            py.importlib.import_module('casadi');
            py.importlib.import_module('acados_template');
            pyModule = py.importlib.import_module('sun_acados_nmpc');
        catch err
            pe = pyenv;
            configuredPython = '<not configured>';
            if isfield(par.sun, 'pythonExecutable') ...
                    && strlength(string(par.sun.pythonExecutable)) > 0
                configuredPython = char(par.sun.pythonExecutable);
            end
            error(['sun_nmpc cannot import its Python/acados dependencies.\n' ...
                   'MATLAB is currently using Python:\n  %s\n' ...
                   'Configured acados Python:\n  %s\n' ...
                   'Run install_acados once from the project root. ' ...
                   'It creates/uses .venv and installs casadi/acados_template.\n' ...
                   'If MATLAB has already loaded the wrong Python, restart ' ...
                   'MATLAB after setup and run main again.\n' ...
                   'Original import error:\n%s'], ...
                  char(pe.Executable), configuredPython, err.message);
	        end
	    end

    sunConfigureAcadosSolver(pyModule, par);
    pySolver = pyModule;
end

function sunAssertAcadosInstallReady(par)

    acadosDir = char(par.sun.acadosSourceDir);
    requiredPaths = [
        string(fullfile(acadosDir, "interfaces", "acados_template"))
        string(fullfile(acadosDir, "lib", "link_libs.json"))
    ];

    missing = strings(0,1);
    for i = 1:numel(requiredPaths)
        path = requiredPaths(i);
        if exist(path, 'file') ~= 2 && exist(path, 'dir') ~= 7
            missing(end+1,1) = path; %#ok<AGROW>
        end
    end

    requiredLibs = ["libacados", "libblasfeo", "libhpipm"];
    for i = 1:numel(requiredLibs)
        libPattern = fullfile(acadosDir, "lib", requiredLibs(i) + ".*");
        if isempty(dir(libPattern))
            missing(end+1,1) = string(libPattern); %#ok<AGROW>
        end
    end

    if ~isempty(missing)
        error(['sun_nmpc acados installation is incomplete.\n' ...
               'Configured acados directory:\n  %s\n' ...
               'Missing required path(s):\n  %s\n' ...
               'Run install_acados once from the project root on this computer.\n' ...
               'If .acados/acados already exists from a failed/partial setup, ' ...
               'rerun install_acados after deleting or rebuilding that checkout.'], ...
              acadosDir, strjoin(missing, newline + "  "));
    end
end

function sunConfigureAcadosSolver(pyModule, par)

    G1 = sunAllocationMatrix(par);

    if isfield(par.sun, 'acadosCodegenDir')
        codegenDir = par.sun.acadosCodegenDir;
    else
        codegenDir = fullfile(tempdir, "uav_sun_acados_codegen");
    end

    pyModule.configure(pyargs( ...
        'n_horizon', int32(par.sun.N), ...
        'dt', double(par.sun.dt), ...
        'mass', double(par.m), ...
        'gravity', double(par.g), ...
        'inertia_diag', py.numpy.array(diag(par.J)'), ...
        'allocation_matrix', py.numpy.array(G1), ...
        'u_min', py.numpy.array(par.allocation.uMin'), ...
        'u_max', py.numpy.array(par.allocation.uMax'), ...
        'omega_max', py.numpy.array(par.sun.omegaMax'), ...
        'aero_enabled', logical(par.aero.enabled), ...
        'aero_kd', py.numpy.array(par.aero.kd(:)'), ...
        'aero_kh', double(par.aero.kh), ...
        'q_xi', py.numpy.array(par.sun.Qxi), ...
        'q_v', py.numpy.array(par.sun.Qv), ...
        'q_q', py.numpy.array(par.sun.Qq), ...
        'q_omega', py.numpy.array(par.sun.QOmega), ...
        'q_u', py.numpy.array(par.sun.Qu), ...
        'code_export_dir', char(codegenDir)));
end

function sunConfigurePythonForAcados(par)

    if ~isfield(par.sun, 'pythonExecutable') ...
            || strlength(string(par.sun.pythonExecutable)) == 0
        return;
    end

    desiredPython = char(par.sun.pythonExecutable);
    if exist(desiredPython, 'file') ~= 2
        error(['Configured acados Python does not exist:\n  %s\n' ...
               'Set ACADOS_PYTHON to a valid python3, or run ' ...
               'install_acados to create the project .venv.'], ...
              desiredPython);
    end

    pe = pyenv;
    if string(pe.Status) == "NotLoaded"
        pyenv('Version', desiredPython, 'ExecutionMode', 'InProcess');
        return;
    end

    currentPython = char(pe.Executable);
    if strcmp(currentPython, desiredPython)
        return;
    end

    try
        py.importlib.import_module('casadi');
        py.importlib.import_module('acados_template');
    catch err
        error(['MATLAB has already loaded a Python environment that cannot ' ...
               'import Sun NMPC dependencies.\n' ...
               'Current Python:\n  %s\n' ...
               'Prepared Python:\n  %s\n' ...
               'Restart MATLAB, then before running main execute:\n' ...
               '  pyenv(''Version'', ''%s'', ''ExecutionMode'', ''InProcess'')\n' ...
               'Or run setup_acados_python to install dependencies into ' ...
               'the current Python.\nOriginal import error:\n%s'], ...
              currentPython, desiredPython, desiredPython, err.message);
    end
end

function refs = sunNMPCBuildReferences(traj, t, N, h, par)

    % Paper Eq. (10): supplies x_r = [xi_r; xidot_r; q_r; Omega_r] and u_r
    % as four actuator thrusts. The prediction reference may run past par.Tend
    % by N*h, but the simulation and logged tracking error still stop exactly
    % at par.Tend.
    refs.p = zeros(3, N + 1);
    refs.v = zeros(3, N + 1);
    refs.R = zeros(3, 3, N + 1);
    refs.q = zeros(4, N + 1);
    refs.Omega = zeros(3, N + 1);
    refs.alpha = zeros(3, N + 1);
    refs.tau = zeros(3, N + 1);
    refs.T = zeros(1, N + 1);
    refs.u = zeros(4, N + 1);

    for k = 1:N+1
        tk = t + (k-1)*h;
        ref = predictionReferenceAtTime(traj, tk, par);
        ff = sunAeroFlatnessReference(ref, par);

        refs.p(:,k) = ref.p;
        refs.v(:,k) = ref.v;
        refs.R(:,:,k) = ff.R;
        refs.q(:,k) = rotm2quat(ff.R).';
        refs.Omega(:,k) = ff.Omega;
        refs.alpha(:,k) = ff.alpha;
        refs.T(k) = ff.T;
    end

    for k = 1:N+1
        refs.tau(:,k) = par.J*refs.alpha(:,k) ...
            + cross(refs.Omega(:,k), par.J*refs.Omega(:,k));
        refs.u(:,k) = sunDirectAllocation([refs.T(k); refs.tau(:,k)], par);
    end
end

function ref = predictionReferenceAtTime(traj, tk, par)

    % Receding-horizon controllers should not see a terminal stop just because
    % tk lies past the simulated logging interval. For trajectories created by
    % this factory, evalPredict extrapolates the analytic curve with the terminal
    % angular frequency. The fallback is only for externally supplied trajectories.
    if isfield(traj, 'evalPredict')
        ref = traj.evalPredict(tk);
    else
        ref = traj.eval(min(tk, par.Tend));
    end
end

function xVec = sunNMPCStateVector(x)

    xVec = [x.p;
            rotm2quat(x.R).';
            x.v;
            x.Omega];
end

function u = controlAllocation(mu, Rd, par)

    muCmd = [-mu(1); mu(2:4)];
    u.Rd = Rd;
    u.actuator = allocateActuator(muCmd, par);

    % Actuator dynamics are neglected here: after saturation, the actuator
    % vector is multiplied by the allocation matrix to obtain the wrench
    % applied to the rigid-body model.
    u.muAllocated = par.allocation.B*u.actuator(:);
    u.T = -u.muAllocated(1);
    u.tau = u.muAllocated(2:4);
end

function u = sunDFBCAllocation(mu, Rd, par)

    if par.allocation.method ~= "wls"
        u = controlAllocation(mu, Rd, par);
        return;
    end

    muCmd = [-mu(1); mu(2:4)];
    B = par.allocation.B;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    Wq = par.sun.allocationW;
    Wv = diag(sqrt(diag(Wq)));
    Wu = eye(numel(lb));
    ud = zeros(numel(lb),1);
    gamma = double(getStructField(par.allocation, 'gamma', 1e6));
    u0 = (lb + ub)/2;
    W0 = zeros(numel(lb),1);

    actuator = wls_alloc(B, muCmd(:), lb, ub, Wv, Wu, ud, gamma, u0, W0, 100);
    actuator = min(max(actuator(:), lb), ub);

    u.Rd = Rd;
    u.actuator = actuator;
    u.muAllocated = B*u.actuator(:);
    u.T = -u.muAllocated(1);
    u.tau = u.muAllocated(2:4);
end

function actuator = allocateActuator(muCmd, par)

    switch par.allocation.method
        case "pinv"
            actuator = allocateActuatorPinv(muCmd, par);
        case "wls"
            actuator = allocateActuatorWLS(muCmd, par);
        otherwise
            error("Unknown allocation method.");
    end
end

function actuator = allocateActuatorPinv(muCmd, par)

    actuator = min(max(par.allocation.B\muCmd(:), ...
        par.allocation.uMin(:)), par.allocation.uMax(:));
end

function actuator = allocateActuatorWLS(muCmd, par)

    % Unified QCAT WLS allocation on the benchmark wrench convention
    % B*u=[-T;tau]. This follows control_allocation/test.m: Wv=I, Wu=I,
    % ud=0, gamma large, and u0 at the actuator-box midpoint.
    B = par.allocation.B;
    lb = par.allocation.uMin(:);
    ub = par.allocation.uMax(:);
    [k, m] = size(B);
    Wv = eye(k);
    Wu = eye(m);
    ud = zeros(m,1);
    gamma = double(getStructField(par.allocation, 'gamma', 1e6));
    u0 = (lb + ub)/2;
    W0 = zeros(numel(lb),1);

    actuator = wls_alloc(B, muCmd(:), lb, ub, Wv, Wu, ud, gamma, u0, W0, 100);
    actuator = min(max(actuator(:), lb), ub);
end

function u = controlAllocationPX4Normalized(muNormCmd, Rd, par)

    % muNormCmd is [Fz; Mx; My; Mz], matching B_px4_norm.
    u.Rd = Rd;
    u.actuator = min(max(par.allocation.B_px4_norm\muNormCmd(:), 0), 1);

    % Actuator dynamics are neglected here: after saturation, the normalized
    % actuator vector is multiplied by the physical PX4 allocation matrix.
    u.muAllocated = par.allocation.B_px4*u.actuator(:);
    u.T = -u.muAllocated(1);
    u.tau = u.muAllocated(2:4);
end

function G1 = sunAllocationMatrix(par)

    % Sun uses [T; tau] = G1*u. The benchmark allocation matrix stores
    % [-T; tau], so keep the original geometry and only change the first sign.
    G1 = [-par.allocation.B(1,:); par.allocation.B(2:4,:)];
end

function uControl = sunActuatorToControl(actuator, Rd, par)

    actuator = actuator(:);
    mu = sunAllocationMatrix(par)*actuator;

    uControl.Rd = Rd;
    uControl.actuator = actuator;
    uControl.muAllocated = [-mu(1); mu(2:4)];
    uControl.T = mu(1);
    uControl.tau = mu(2:4);
end

function actuator = sunDirectAllocation(mu, par)

    % Used only to form rotor-thrust reference inputs for Sun NMPC's OCP.
    % Controllers that command a physical wrench [T;tau] use the unified
    % controlAllocation path instead.
    % Sun Eq. (29): direct inversion with G2/G3 omitted.
    actuator = sunAllocationMatrix(par)\mu(:);
end

function u = controllerSunDFBC(x, ref, traj, t, par)

    % Algorithm model/control law:
    %   Sun DFBC uses the same plant model as NMPC but computes the command
    %   explicitly:
    %     accD = Kxi*(xi_d-xi) + Kv*(v_d-v) + xi_ddot_d,
    %     T*R_d*e3 = m*(g*e3-accD) + R*f_a^B,
    %     alphaD = Kq_red*q_red + kq_yaw*q_yaw
    %              + KOmega*(OmegaR-Omega) + alphaR,
    %     tauD = J*alphaD + Omega x J*Omega.
    cmd = sunDFBCCommand(x, ref, traj, t, par);
    u = sunActuatorToControl(cmd.actuator, cmd.Rd, par);
    u.OmegaD = cmd.OmegaR;
    u.alphaD = cmd.alphaR;
end

function u = controllerSunDFBCINDI(x, ref, traj, t, par)

    persistent st

    % Algorithm model/control law:
    %   Outer loop is Sun DFBC. Inner loop applies Sun Eq. (32)-(35):
    %     tauCmd = tau_f + J*(alphaD-alpha_f),
    %   using filtered achieved actuator/moment feedback from the benchmark
    %   allocation state.
    cmd = scheduledSample("sun_dfbc_indi_outer", t, ...
        par.control.innerPeriod, par, ...
        @() sunDFBCCommand(x, ref, traj, t, par));
    [u, st] = sunINDIActuatorControl(x, cmd, t, par, st);
    u.OmegaD = cmd.OmegaR;
    u.alphaD = cmd.alphaR;
end

function u = controllerSunNMPCINDI(x, ref, traj, t, par)

    persistent st

    % Algorithm model/control law:
    %   Outer loop is Sun NMPC actuator command. The actuator command is
    %   converted to achievable [T; alpha], then the same Sun Eq. (32)-(35)
    %   INDI inner loop computes the final constrained actuator command.
    uMpc = controllerSunNMPC(x, ref, traj, t, par);
    cmd = sunCommandFromActuator(uMpc.actuator, uMpc.Rd, x, par);
    [u, st] = sunINDIActuatorControl(x, cmd, t, par, st);

    u.sunNMPCCached = uMpc.sunNMPCCached;
    u.sunNMPCSolved = uMpc.sunNMPCSolved;
    u.sunNMPCStatusCode = uMpc.sunNMPCStatusCode;
    u.sunNMPCExitflag = uMpc.sunNMPCExitflag;
    u.sunNMPCSolveTime = uMpc.sunNMPCSolveTime;
    u.OmegaD = uMpc.OmegaD;
    u.alphaD = uMpc.alphaD;
end

function cmd = sunDFBCCommand(x, ref, ~, t, par)

    persistent st

    % Sun et al. Eq. (13): desired acceleration from PD position feedback:
    %   accD = Kxi*(xi_d-xi) + Kv*(v_d-v) + xi_ddot_d.
    xiErr = ref.p - x.p;
    vErr = ref.v - x.v;
    accD = par.sun.Kxi*xiErr + par.sun.Kv*vErr + ref.a;

    % Sun et al. Eq. (14)-(17), converted from the paper's convention where
    % z_B is the thrust direction to this NED/FRD model where R*e3 is opposite
    % thrust: a = g*e3 - T/m*R*e3 + R*f_a^B/m. Thus
    %   T*Rd*e3 = m*(g*e3 - accD) + R*f_a^B.
    fAeroWorld = x.R*sunAeroForceBody(x.R, x.v, par);
    thrustAxisForce = par.m*(par.g*par.e3 - accD) + fAeroWorld;
    [Rd, T] = sunDesiredAttitudeFromThrustVector(thrustAxisForce, ref.psi);

    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    if resetState || ~isfield(st, 'T')
        TForFlatness = T;
    else
        TForFlatness = st.T;
    end

    % Sun et al. Eq. (18)-(24): use the current attitude/angular velocity and
    % the currently applied collective thrust. In this direct-actuator
    % benchmark, the previous allocated thrust is the available thrust
    % measurement; the first sample falls back to the newly requested T.
    [OmegaR, alphaR] = sunFlatnessReferenceRates(x, ref, TForFlatness, par);

    % Sun et al. Eq. (25): for B-to-I attitude, use q_e = q^{-1} \otimes q_d
    % so the reduced/yaw tangent vectors below are body-local errors.
    qd = rotm2quat(Rd).';
    q = rotm2quat(x.R).';
    qe = quatmultiply([q(1), -q(2:4).'], qd.').';
    qe = qe/norm(qe);

    % Sun et al. Eq. (26)-(27): split reduced-attitude and yaw errors.
    den = sqrt(qe(1)^2 + qe(4)^2);
    qRed = [qe(1)*qe(2) - qe(3)*qe(4);
            qe(1)*qe(3) + qe(2)*qe(4);
            0]/den;
    qYaw = [0; 0; qe(4)]/den;

    % Sun et al. Eq. (28): tilt-prioritized attitude control.
    yawSign = 1;
    if qe(1) < 0
        yawSign = -1;
    end

    % Sun Eq. (28):
    %   alphaD = KqRed*qRed + kqYaw*sign(qe_w)*qYaw
    %            + KOmega*(OmegaR-Omega) + alphaR.
    alphaD = par.sun.KqRed*qRed + par.sun.kqYaw*yawSign*qYaw ...
           + par.sun.KOmega*(OmegaR - x.Omega) + alphaR;

    % Rigid-body inversion:
    %   tauDesired = J*alphaD + Omega x J*Omega.
    tauDesired = par.J*alphaD + cross(x.Omega, par.J*x.Omega);
    % Sun Eq. (31): constrained allocation from desired [T;tau] to rotor
    % thrust commands. Use the benchmark-unified allocation implementation;
    % with par.allocation.method="wls" this is the QCAT/test.m WLS setup.
    uAlloc = sunDFBCAllocation([T; tauDesired], Rd, par);

    cmd.actuator = uAlloc.actuator;
    % Sun Eq. (32): retrieve the actually achievable collective thrust and
    % angular acceleration after the Eq. (31) constrained allocation.
    cmd.T = uAlloc.T;
    cmd.alpha = par.J \ (uAlloc.tau ...
        - cross(x.Omega, par.J*x.Omega));
    cmd.Rd = Rd;
    cmd.OmegaR = OmegaR;
    cmd.alphaR = alphaR;
    cmd.alphaD = alphaD;

    st.T = uAlloc.T;
    st.t = t;
end

function cmd = sunCommandFromActuator(actuator, Rd, x, par)

    % Convert an already selected actuator vector back to the command form
    % used by Sun INDI:
    %   alpha = J \ (tau - Omega x J*Omega).
    u = sunActuatorToControl(actuator, Rd, par);
    cmd.actuator = u.actuator;
    cmd.T = u.T;
    cmd.alpha = par.J \ (u.tau - cross(x.Omega, par.J*x.Omega));
    cmd.Rd = Rd;
end

function [Rd, T] = sunDesiredAttitudeFromThrustVector(thrustAxisForce, psi)

    % Sun Eq. (14)-(17), expressed in this NED/FRD benchmark convention.
    % No fallback is applied: zero thrust or yaw-axis degeneracy is left as
    % the paper's mapping produces it.
    T = norm(thrustAxisForce);
    zBd = thrustAxisForce/T;
    xCd = [cos(psi); sin(psi); 0];
    yBd = cross(zBd, xCd)/norm(cross(zBd, xCd));
    xBd = cross(yBd, zBd);
    Rd = [xBd, yBd, zBd];
end

function [u, st] = sunINDIActuatorControl(x, cmd, t, par, st)

    % Sun et al. Eq. (32)-(35). The MATLAB plant has no motor-speed state;
    % since the benchmark actuator variable is already u=c_t*omega^2, the
    % previous allocated actuator vector stands in for filtered rotor-thrust
    % feedback. The unknown d_tau in the paper is the plant disturbance here
    % and is not predicted or canceled.

    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    if resetState
        st = struct;
        st.omegaFilter = initSecondOrderLPF(x.Omega);
        st.actuatorFilter = initSecondOrderLPF(cmd.actuator);
        st.omegaF = x.Omega;
        st.nextRateTime = t;
    end

    rateDue = resetState || t + 0.5*par.dt >= st.nextRateTime;
    if ~rateDue
        u = st.uHold;
        return;
    end

    if resetState
        omegaDotF = zeros(3,1);
        actuatorF = cmd.actuator;
    else
        h = max(t - st.t, par.sun.indiPeriod);
        rawActuator = st.actuator;
        [omegaF, st.omegaFilter] = secondOrderButterworthLPF( ...
            x.Omega, st.omegaFilter, h, par.sun.innerLoopFilterCutoffHz);
        [actuatorF, st.actuatorFilter] = secondOrderButterworthLPF( ...
            rawActuator, st.actuatorFilter, h, par.sun.innerLoopFilterCutoffHz);
        omegaDotF = (omegaF - st.omegaF)/h;
        st.omegaF = omegaF;
    end

    % Filtered achieved wrench from filtered actuator thrusts:
    %   muF = G1*uF = [TF; tauF].
    muF = sunAllocationMatrix(par)*actuatorF;
    tauF = muF(2:4);

    % Sun Eq. (33)-(35) in benchmark wrench variables:
    %   mu = [Tcmd; tauF + J*(alphaCmd-omegaDotF)].
    mu = zeros(4,1);
    mu(1) = cmd.T;
    mu(2:4) = tauF + par.J*(cmd.alpha - omegaDotF);

    % Keep Sun's INDI timing: Eq. (33)-(35) first forms the commanded wrench
    % increment from the Eq. (32) achievable values. The final actuator solve
    % is the benchmark-unified allocation layer, because this plant directly
    % commands rotor thrusts rather than motor speed or ESC throttle.
    u = controlAllocation(mu, cmd.Rd, par);

    st.actuator = u.actuator;
    st.uHold = u;
    st.nextRateTime = advanceSampleTime( ...
        st.nextRateTime, t, par.sun.indiPeriod, par.dt);
    st.t = t;
end

function [OmegaR, alphaR] = sunFlatnessReferenceRates(x, ref, T, par)

    ref = completeReferenceDerivatives(ref);

    b1 = x.R(:,1);
    b2 = x.R(:,2);
    b3 = x.R(:,3);       % body z-down axis; thrust acceleration is -T/m*b3.
    omegaWorld = x.R*x.Omega;

    % Sun Eq. (18)-(24), rewritten for the NED plant
    %   a = g*e3 - T/m*b3.
    % Therefore m*j = -Tdot*b3 - T*(Omega x b3).
    TDot = -par.m*dot(ref.j, b3);
    hOmega = (-par.m*ref.j - TDot*b3)/T;
    OmegaR = [-dot(hOmega, b2);
               dot(hOmega, b1);
               ref.psiDot*dot(par.e3, b3)];

    % Differentiating again:
    % m*s = -Tddot*b3 - 2*Tdot*hOmega
    %       - T*(alpha x b3 + Omega x hOmega).
    TDDot = -par.m*dot(ref.s, b3) ...
        - par.m*dot(cross(omegaWorld, b3), ref.j);
    hAlpha = -(par.m/T)*ref.s ...
        - cross(omegaWorld, hOmega) ...
        - 2*(TDot/T)*hOmega ...
        - (TDDot/T)*b3;
    alphaR = [-dot(hAlpha, b2);
               dot(hAlpha, b1);
               ref.psiDDot*dot(par.e3, b3)];
end

function u = controllerLu(x, ref, traj, t, par)

    persistent st

    % Lu et al. on-manifold MPC, quadrotor case.
    %
    % Paper-to-code convention:
    %   Lu Eq. (14)-(16) uses state x=(p,v,R) in M=R3 x R3 x SO(3)
    %   and input u=[aT; omega]. In this benchmark R maps body to NED, so
    %   the same model form is
    %       p_dot = v,
    %       v_dot = g*e3 - aT*R*e3,
    %       R_dot = R*hat(omega).
    %
    %   The MPC variable is the input error delta u = u - u_d from Lu
    %   Eq. (13). Therefore the command below is
    %       [aT_cmd; Omega_cmd] = [aT_d; Omega_d] + delta u_0.
    %
    %   par.lu.solvePeriod is the MPC refresh period Tc. par.lu.dt is the
    %   OCP grid interval dt_ocp = h/N. The shared MPC setup follows Sun
    %   Table I: dt_ocp = 50 ms and N = 20, so h = 1.0 s.
    %
    %   Special handling: Lu's experiment sends aT and body-rate omega to a
    %   low-level body-rate controller. This benchmark uses a Lu-local
    %   PX4-style PID rate adapter at 250 Hz, then sends physical
    %   [m*aT_cmd; tau] through the normal benchmark allocation.
    %   Consequently, the Lu QP itself is almost independent of aircraft
    %   mass/inertia: mass appears through the thrust-acceleration bound and
    %   force conversion; inertia appears in the physical rate-loop gains.
    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    solveDue = resetState || t + 0.5*par.dt >= st.nextSolveTime;

    if solveDue
        % Reference input on M = R3 x R3 x SO(3):
        %   u_d = [aT_d; Omega_d].
        [Rd, aTd, OmegaD] = referenceInputOnManifold(ref, par);

        % Lu Eq. (13): solve for the first input-error command delta u_0.
        refs = onManifoldMPCReferences(ref, traj, t, par);
        du = solveOnManifoldMPC(x, refs, par);

        % Apply the MPC input error:
        %   [aT_cmd; Omega_cmd] = [aT_d; Omega_d] + delta u_0.
        aTCmd = aTd + du(1);
        OmegaCmd = OmegaD + du(2:4);
        % Physical input bounds:
        %   0 <= aT_cmd <= Tmax/m, ||Omega_cmd||_inf <= omegaMax.
        aTCmd = min(max(aTCmd, 0), par.Tmax/par.m);
        OmegaCmd = saturateVector(OmegaCmd, par.lu.omegaMax);

        st.aTCmd = aTCmd;
        st.OmegaCmd = OmegaCmd;
        st.Rd = Rd;
        st.nextSolveTime = t + par.lu.solvePeriod;
    else
        aTCmd = st.aTCmd;
        OmegaCmd = st.OmegaCmd;
        Rd = st.Rd;
    end

    % Lu Eq. (14)-(16) outputs thrust acceleration and body rate. The
    % Lu-local rate adapter below tracks Omega_cmd and returns physical
    % torque; allocation is the same physical [T;tau] path used by the other
    % non-PX4 controllers:
    %   T = m*aT_cmd.
    tau = luMpcRateLoop(x, OmegaCmd, t, par);
    u = controlAllocation([par.m*aTCmd; tau], Rd, par);
    u.OmegaD = OmegaCmd;

    st.t = t;
end

function tau = luMpcRateLoop(x, OmegaCmd, t, par)

    persistent st

    % Lu-local rate adapter model/control law:
    %   eOmega = OmegaCmd - Omega
    %   tau = Omega x J*Omega + rateP*eOmega + integral(rateI*eOmega)
    %         - rateD*OmegaDot + rateFF*OmegaCmd.
    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    if resetState
        st.rateInt = zeros(3,1);
        st.prevOmegaRate = x.Omega;
        st.tau = zeros(3,1);
        st.rateT = t - par.lu.ratePeriod;
        st.nextRateTime = t;
        st.t = t - par.dt;
    end

    rateDue = t + 0.5*par.dt >= st.nextRateTime;
    if rateDue
        h = max(t - st.rateT, par.lu.ratePeriod);
        % Measured angular acceleration for the D term.
        angularAccel = (x.Omega - st.prevOmegaRate)/max(h, eps);
        % Body-rate error: eOmega = OmegaCmd - Omega.
        rateError = OmegaCmd - x.Omega;

        rateP = matrix3(getStructField(par.lu, ...
            'rateP', par.J*par.KOmega), 'par.lu.rateP');
        rateI = matrix3(getStructField(par.lu, ...
            'rateI', zeros(3)), 'par.lu.rateI');
        rateD = matrix3(getStructField(par.lu, ...
            'rateD', zeros(3)), 'par.lu.rateD');
        rateFF = matrix3(getStructField(par.lu, ...
            'rateFF', zeros(3)), 'par.lu.rateFF');
        intLimit = vector3(getStructField(par.lu, ...
            'rateIntLimit', inf(3,1)));

        % Integral state with componentwise saturation.
        st.rateInt = st.rateInt + rateI*rateError*h;
        st.rateInt = min(max(st.rateInt, -intLimit), intLimit);

        % Physical moment command:
        %   tau = Omega x J*Omega + P*eOmega + I - D*OmegaDot + FF*OmegaCmd.
        tau = cross(x.Omega, par.J*x.Omega) ...
            + rateP*rateError + st.rateInt ...
            - rateD*angularAccel + rateFF*OmegaCmd;

        st.tau = tau;
        st.prevOmegaRate = x.Omega;
        st.rateT = t;
        st.nextRateTime = advanceSampleTime( ...
            st.nextRateTime, t, par.lu.ratePeriod, par.dt);
    else
        tau = st.tau;
    end

    st.t = t;
end

function u = controllerGeometricINDI(x, ref, t, par)

    persistent st

    % Algorithm model/control law:
    %   Plant model:
    %     v_dot = g*e3 - T/m*b3,  J*Omega_dot = tau - Omega x J*Omega.
    %   Outer loop:
    %     aCmd = Kp*(p_d-p) + Kv*(v_d-v) + a_d.
    %   Incremental thrust-vector law, GINDI Eq. (55):
    %     T*b3 = T0*b30 - m*(aCmd-vDot0).
    %   Attitude/rate law:
    %     OmegaDotCmd = Ktheta*Log(R'*Rd)
    %                   + Komega*(OmegaRef-OmegaF) + alphaRef
    %     tau = tau0F + J*(OmegaDotCmd-OmegaDot0F).
    ref = completeReferenceDerivatives(ref);
    % Position errors: ep = p_d-p, ev = v_d-v.
    ep = ref.p - x.p;
    ev = ref.v - x.v;
    % GINDI Eq. (56): aCmd = Kp*ep + Kv*ev + a_d.
    aCmd = par.geometric_indi.Kp*ep + par.geometric_indi.Kv*ev + ref.a;

    resetState = isempty(st) || t <= par.dt/2 || t <= st.t;
    if resetState
        % First sample uses direct inversion:
        %   thrustAxisForce = m*(g*e3-aCmd), T = ||thrustAxisForce||.
        thrustAxisForce = par.m*(par.g*par.e3 - aCmd);
        TForRates = norm(thrustAxisForce);
        cmd = geometricINDIReferenceCommand( ...
            x, ref, thrustAxisForce, TForRates, par);
        T = cmd.T;
        Rd = cmd.Rd;
        omegaRefBody = cmd.omegaRefBody;
        alphaRefBody = cmd.alphaRefBody;

        rErr = LogSO3(x.R' * Rd);
        omegaErr = omegaRefBody - x.Omega;
        % Initial angular acceleration command:
        %   OmegaDotCmd = Ktheta*rErr + Komega*omegaErr + alphaRefBody.
        OmegaDotCmd = par.geometric_indi.Ktheta*rErr ...
                    + par.geometric_indi.Komega*omegaErr ...
                    + alphaRefBody;
        % Direct rigid-body inversion for the first sample:
        %   tau = Omega x J*Omega + J*OmegaDotCmd.
        tau = cross(x.Omega, par.J*x.Omega) + par.J*OmegaDotCmd;
        u = controlAllocation([T; tau], Rd, par);
        u.OmegaD = omegaRefBody;
        u.alphaD = alphaRefBody;

        st = initGeometricINDIState(x, u, t, par);
        return;
    end

    h = max(t - st.t, par.dt);
    % Filtered feedback for the incremental terms. Direct finite differences
    % of noisy v/Omega feedthrough as noise/h and destabilize the INDI update.
    rawOmegaF = x.Omega;
    [rawVDot0, rawOmegaDot0] = rigidBodyRates( ...
        x.R, x.v, x.Omega, st.uApplied, par, t);
    rawT_b_z0 = st.T * x.R*par.e3;
    rawTau0 = st.tau;

    [vDot0, st.vDotFilter] = secondOrderButterworthLPF( ...
        rawVDot0, st.vDotFilter, h, par.geometric_indi.outerLoopFilterCutoffHz);
    [omegaF, st.omegaFilter] = secondOrderButterworthLPF( ...
        rawOmegaF, st.omegaFilter, h, par.geometric_indi.innerLoopFilterCutoffHz);
    [OmegaDot0, st.omegaDotFilter] = secondOrderButterworthLPF( ...
        rawOmegaDot0, st.omegaDotFilter, h, par.geometric_indi.innerLoopFilterCutoffHz);
    [T_b_z0, st.thrustAxisForceFilter] = secondOrderButterworthLPF( ...
        rawT_b_z0, st.thrustAxisForceFilter, h, par.geometric_indi.outerLoopFilterCutoffHz);
    [tau0, st.tauFilter] = secondOrderButterworthLPF( ...
        rawTau0, st.tauFilter, h, par.geometric_indi.innerLoopFilterCutoffHz);

    % GINDI Eq. (55): incremental virtual thrust-vector command:
    %   T_b_z = T0*b30 - m*(aCmd-vDot0).
    T_b_z = T_b_z0 - par.m*(aCmd - vDot0);
    cmd = geometricINDIReferenceCommand( ...
        x, ref, T_b_z, norm(T_b_z0), par);
    T = cmd.T;
    Rd = cmd.Rd;
    omegaRefBody = cmd.omegaRefBody;
    alphaRefBody = cmd.alphaRefBody;

    rErr = LogSO3(x.R' * Rd);
    % GINDI Eq. (61): commanded angular acceleration.
    OmegaDotCmd = par.geometric_indi.Ktheta*rErr ...
                + par.geometric_indi.Komega*(omegaRefBody - omegaF) ...
                + alphaRefBody;

    % GINDI Eq. (60): incremental moment command.
    tau = tau0 + par.J*(OmegaDotCmd - OmegaDot0);

    u = controlAllocation([T; tau], Rd, par);
    u.OmegaD = omegaRefBody;
    u.alphaD = alphaRefBody;

    st = updateGeometricINDIState(st, x, u, t);
end

function cmd = geometricINDIReferenceCommand( ...
        x, ref, thrustAxisForce, TForRates, par)

    % Sun Eq. (14)-(17) gives the thrust-axis/yaw attitude construction used
    % by this GINDI path. Reference angular velocity/acceleration use Sun
    % Eq. (18)-(24).
    T = norm(thrustAxisForce);
    if T < 1e-9
        % First singularity: the thrust axis is not defined. Keep the
        % current attitude; desired angular velocity and acceleration are 0.
        Rd = x.R;
        omegaRefBody = zeros(3,1);
        alphaRefBody = zeros(3,1);
    else
        b3d = thrustAxisForce/T;
        xC = [cos(ref.psi); sin(ref.psi); 0];
        if norm(cross(b3d, xC)) < 1e-8
            % Second singularity: use the same fallback heading as the
            % initial-attitude construction.
            xC = [0; 1; 0];
        end
        Rd = attitudeFromUnitBodyZAndHeading(b3d, xC);
        if TForRates < 1e-9
            % The flatness rate map contains 1/T. If the measured/reference
            % thrust used for rates is singular, keep rates at zero.
            omegaRefBody = zeros(3,1);
            alphaRefBody = zeros(3,1);
        else
            [omegaRefBody, alphaRefBody] = sunFlatnessReferenceRates( ...
                x, ref, TForRates, par);
        end
    end

    cmd.Rd = Rd;
    cmd.T = T;
    cmd.omegaRefBody = omegaRefBody;
    cmd.alphaRefBody = alphaRefBody;
end

function st = initGeometricINDIState(x, u, t, par)

    [aApplied, omegaDotApplied] = rigidBodyRates( ...
        x.R, x.v, x.Omega, u, par, t);
    st.vDotFilter = initSecondOrderLPF(aApplied);
    st.omegaFilter = initSecondOrderLPF(x.Omega);
    st.omegaDotFilter = initSecondOrderLPF(omegaDotApplied);
    st.thrustAxisForceFilter = initSecondOrderLPF(u.T*x.R*par.e3);
    st.tauFilter = initSecondOrderLPF(u.tau);
    st = updateGeometricINDIState(st, x, u, t);
end

function st = updateGeometricINDIState(st, x, u, t)

    st.v = x.v;
    st.R = x.R;
    st.Omega = x.Omega;
    st.T = u.T;
    st.tau = u.tau;
    st.uApplied = u;
    st.t = t;
end

function [Rd, aT, OmegaD] = referenceInputOnManifold(ref, par)

    % Lu Eq. (6), (13), and (15): the MPC reference is
    % x_d=(p_d,v_d,R_d), u_d=[aT_d;Omega_d]. The flat output derivatives are
    % only used to construct these quantities; alpha_d is not an MPC input.
    ff = luFlatnessReference(ref, par);

    Rd = ff.R;
    aT = ff.T/par.m;
    OmegaD = ff.Omega;
end

function refs = onManifoldMPCReferences(ref, traj, t, par)

    % Build the reference sequence x_d(k), u_d(k) used in Lu Eq. (6) and
    % Eq. (13). The reference trajectory is evaluated analytically where the
    % factory provides derivatives; no finite-difference reference rates are
    % introduced here. Each knot stores exactly what Lu's quadrotor MPC uses:
    % x_d=(p_d,v_d,R_d), u_d=[aT_d;Omega_d].
    N = par.lu.N;
    refs.p = zeros(3, N + 1);
    refs.v = zeros(3, N + 1);
    refs.R = zeros(3, 3, N + 1);
    refs.aT = zeros(1, N + 1);
    refs.Omega = zeros(3, N + 1);

    ff0 = luFlatnessReference(ref, par);
    refs.p(:,1) = ref.p;
    refs.v(:,1) = ref.v;
    refs.R(:,:,1) = ff0.R;
    refs.aT(1) = ff0.c;
    refs.Omega(:,1) = ff0.Omega;

    h = par.lu.dt;
    for k = 2:N+1
        tk = t + (k-1)*h;
        refK = predictionReferenceAtTime(traj, tk, par);

        ff = luFlatnessReference(refK, par);
        refs.p(:,k) = refK.p;
        refs.v(:,k) = refK.v;
        refs.R(:,:,k) = ff.R;
        refs.aT(k) = ff.c;
        refs.Omega(:,k) = ff.Omega;
    end
end

function du0 = solveOnManifoldMPC(x, refs, par)

    % Lu Eq. (13): standard Euclidean MPC over error coordinates.
    %
    % Error-state definition, matching Eq. (13):
    %   delta x = x boxminus x_d
    %           = [p-p_d; v-v_d; Log(R_d' R)] in this benchmark.
    % The right attitude error is consistent with the SO(3) boxplus
    % R = R_d Exp(delta theta) used by the linearization below.
    %
    % Input-error definition:
    %   delta u = u - u_d = [aT-aT_d; Omega-Omega_d].
    %
    % Linearized error dynamics, Lu Eq. (9):
    %   delta x_{k+1} = A_k delta x_k + B_k delta u_k.
    %
    % Condensed prediction:
    %   delta X = Hx*delta x_0 + Mu*delta U,
    % where delta X stacks k=1..N and delta U stacks k=0..N-1.
    %
    % Condensed cost from Lu Eq. (13):
    %   sum ||delta x_k||_Q^2 + ||delta u_k||_R^2 + ||delta x_N||_P^2
    % becomes
    %   min_deltaU 0.5*deltaU' H deltaU + f' deltaU
    % with the irrelevant global factor of two dropped:
    %   H = Mu' Qbar Mu + Rbar,
    %   f = Mu' Qbar Hx delta x_0.
    %
    % Bounds are exactly Lu Eq. (13):
    %   u_min - u_d(k) <= delta u_k <= u_max - u_d(k).
    N = par.lu.N;
    nx = 9;
    nu = 4;
    h = par.lu.dt;

    dx0 = [x.p - refs.p(:,1);
           x.v - refs.v(:,1);
           LogSO3(refs.R(:,:,1)' * x.R)];

    Hx = zeros(nx*N, nx);
    Mu = zeros(nx*N, nu*N);
    Aseq = zeros(nx, nx, N);
    Bseq = zeros(nx, nu, N);

    for k = 1:N
        [Aseq(:,:,k), Bseq(:,:,k)] = onManifoldMPCLinearization( ...
            refs.R(:,:,k), refs.aT(k), refs.Omega(:,k), h, par);
    end

    Aprod = eye(nx);
    for i = 1:N
        Aprod = Aseq(:,:,i)*Aprod;
        Hx((i-1)*nx+1:i*nx,:) = Aprod;

        for j = 1:i
            Aj = eye(nx);
            for l = j+1:i
                Aj = Aseq(:,:,l)*Aj;
            end
            Mu((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Aj*Bseq(:,:,j);
        end
    end

    Qbar = kron(eye(N), par.lu.Q);
    Qbar(end-nx+1:end, end-nx+1:end) = par.lu.P;
    Rbar = kron(eye(N), par.lu.R);

    H = Mu'*Qbar*Mu + Rbar;
    f = Mu'*Qbar*Hx*dx0;

    [lb, ub] = onManifoldMPCInputBounds(refs, par);
    du = solveBoxQP(H, f, lb, ub, par.lu.maxQPIt, par.lu.qpTol);
    du0 = du(1:nu);
end

function [Ad, Bd] = onManifoldMPCLinearization(Rd, aTd, OmegaD, h, par)

    % Lu Eq. (10)-(12): A_k = Gx + dt*Gf*df/d(delta x),
    %                    B_k = dt*Gf*df/d(delta u).
    %
    % Manifold-specific part for M=R3 x R3 x SO(3):
    %   R3 position and velocity blocks have Gx=Gf=I.
    %   SO(3) uses Appendix Eq. (23) with v=dt*Omega_d:
    %       Gx_R = Exp(-v),
    %       Gf_R = A(v)' = leftJacobianSO3(v)'.
    %
    % System-specific part from Lu Eq. (16), adapted to this benchmark's
    % body-to-NED R and v_dot = g*e3 - aT*R*e3:
    %   df/d(delta x) =
    %       [0 I 0;
    %        0 0 aT*R*hat(e3);
    %        0 0 0],
    %   df/d(delta u) =
    %       [0 0;
    %        -R*e3 0;
    %        0 I].
    %
    % The sign aT*R*hat(e3) follows from R=R_d Exp(delta theta):
    % d[-aT*R*e3]/d(delta theta) at zero = aT*R*hat(e3).
    nx = 9;
    phi = h*OmegaD;
    Gx = eye(nx);
    Gf = eye(nx);
    Gx(7:9,7:9) = expm(-hat(phi));
    Gf(7:9,7:9) = leftJacobianSO3(phi)';

    dfdx = zeros(nx, nx);
    dfdu = zeros(nx, 4);
    dfdx(1:3,4:6) = eye(3);
    dfdx(4:6,7:9) = aTd*Rd*hat(par.e3);
    dfdu(4:6,1) = -Rd*par.e3;
    dfdu(7:9,2:4) = eye(3);

    Ad = Gx + h*Gf*dfdx;
    Bd = h*Gf*dfdu;
end

function [lb, ub] = onManifoldMPCInputBounds(refs, par)

    % Lu Eq. (13) input-error set delta U_k. The physical command bounds
    % are on u=[aT;Omega], so the QP bounds are shifted by u_d(k).
    N = par.lu.N;
    nu = 4;
    lb = zeros(nu*N,1);
    ub = zeros(nu*N,1);
    omegaMax = par.lu.omegaMax(:);
    uMin = [0; -omegaMax];
    uMax = [par.Tmax/par.m; omegaMax];

    for k = 1:N
        uRef = [refs.aT(k); refs.Omega(:,k)];
        idx = (k-1)*nu+1:k*nu;
        lb(idx) = uMin - uRef;
        ub(idx) = uMax - uRef;
    end
end

function x = solveBoxQP(H, f, lb, ub, maxIt, tol)

    % Solver for the bound-constrained QP in Lu Eq. (13). quadprog is used
    % when available; the projected-gradient fallback is deterministic and
    % keeps the benchmark runnable without extra toolboxes.
    H = 0.5*(H + H') + 1e-9*eye(size(H));
    if exist('quadprog', 'file') == 2
        opts = optimoptions('quadprog', 'Display', 'off');
        [x, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], opts);
        if exitflag > 0
            return;
        end
    end

    L = max(eig(H));
    if ~isfinite(L) || L <= 0
        L = 1;
    end
    step = 1/L;
    x = min(max(-(H\f), lb), ub);

    for k = 1:maxIt
        grad = H*x + f;
        xNext = min(max(x - step*grad, lb), ub);
        if norm(xNext - x, inf) < tol
            x = xNext;
            return;
        end
        x = xNext;
    end
end

%% ========================================================================
%% Flatness attitude map layer
function ff = luFlatnessReference(ref, par)

    ref = completeReferenceDerivatives(ref);

    % Lu's quadrotor model Eq. (14)-(16) has input u=[aT;omega] and does not
    % include aerodynamic-force states. Generate the needed reference input
    % with Sun Eq. (14)-(17)'s analytic flatness attitude map, but without
    % Sun's aero-force iteration.
    thrustAxisForce = par.m*(par.g*par.e3 - ref.a);
    [R, T, Omega, alpha] = sunAttitudeFromThrustDerivatives( ...
        thrustAxisForce, -par.m*ref.j, -par.m*ref.s, ...
        ref.psi, ref.psiDot, ref.psiDDot);

    ff.R = R;
    ff.T = T;
    ff.c = T/par.m;
    ff.Omega = Omega;
    ff.alpha = alpha;
end


function ff = sunAeroFlatnessReference(ref, par)

    ref = completeReferenceDerivatives(ref);

    thrustAxisForce = par.m*(par.g*par.e3 - ref.a);
    [R, T, Omega, alpha] = sunAttitudeFromThrustDerivatives( ...
        thrustAxisForce, -par.m*ref.j, -par.m*ref.s, ...
        ref.psi, ref.psiDot, ref.psiDDot);

    for i = 1:3
        if ~isfield(par, 'aero') || ~par.aero.enabled
            break;
        end

        aeroForceWorld = R*sunAeroForceBody(R, ref.v, par);
        thrustAxisForce = par.m*(par.g*par.e3 - ref.a) + aeroForceWorld;
        [R, T, Omega, alpha] = sunAttitudeFromThrustDerivatives( ...
            thrustAxisForce, -par.m*ref.j, -par.m*ref.s, ...
            ref.psi, ref.psiDot, ref.psiDDot);
    end

    ff.R = R;
    ff.T = T;
    ff.c = T/par.m;
    ff.Omega = Omega;
    ff.alpha = alpha;
end

function [R, T, Omega, alpha] = sunAttitudeFromThrustDerivatives( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot, ...
        psi, psiDot, psiDDot)

    % Sun Eq. (14)-(17), differentiated analytically for NMPC references.
    % The paper mapping is used as-is; singular force/heading cases are not
    % regularized here.
    [zBd, zBdDot, zBdDDot] = unitVectorDerivativesFromVector( ...
        thrustAxisForce, thrustAxisForceDot, thrustAxisForceDDot);
    T = norm(thrustAxisForce);

    xCd = [cos(psi); sin(psi); 0];
    xCdDot = psiDot*[-sin(psi); cos(psi); 0];
    xCdDDot = psiDDot*[-sin(psi); cos(psi); 0] - psiDot^2*xCd;

    yBdRaw = cross(zBd, xCd);
    yBdRawDot = cross(zBdDot, xCd) + cross(zBd, xCdDot);
    yBdRawDDot = cross(zBdDDot, xCd) ...
        + 2*cross(zBdDot, xCdDot) + cross(zBd, xCdDDot);
    [yBd, yBdDot, yBdDDot] = unitVectorDerivativesFromVector( ...
        yBdRaw, yBdRawDot, yBdRawDDot);

    xBd = cross(yBd, zBd);
    xBdDot = cross(yBdDot, zBd) + cross(yBd, zBdDot);
    xBdDDot = cross(yBdDDot, zBd) ...
        + 2*cross(yBdDot, zBdDot) + cross(yBd, zBdDDot);

    R = [xBd, yBd, zBd];
    RDot = [xBdDot, yBdDot, zBdDot];
    RDDot = [xBdDDot, yBdDDot, zBdDDot];

    Omega = vee(R' * RDot);
    alpha = vee(R' * RDDot - hat(Omega)*hat(Omega));
end

%% ========================================================================
%% Quadrotor model layer
% Continuous-time plant model used by both integrators:
%   p_dot = v
%   R_dot = R*hat(Omega)
%   p_ddot = g*e3 - T/m*R*e3 + (f_aero + f_dist)/m
%   J*Omega_dot = tau + tau_dist - Omega x J*Omega
% where p and v are in NED, R maps body to NED, Omega and tau are expressed
% in the body frame, and body +z is the thrust-axis direction opposite the
% generated thrust acceleration.
function xNext = stepModel(x, u, par, t0)

    if nargin < 4
        t0 = 0;
    end

    switch par.integratorName
        case "ode45"
            xNext = stepModelODE45(x, u, par, t0);
        case "lie_rk4"
            xNext = stepModelLieRK4(x, u, par, t0);
        otherwise
            error("Unknown integratorName.");
    end
end

function xNext = stepModelODE45(x, u, par, t0)

    y0 = [x.p; x.v; reshape(x.R, 9, 1); x.Omega];
    opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);
    [~, yHist] = ode45(@(t,y) quadrotorOde(t, y, u, par), ...
        [t0, t0 + par.dt], y0, opts);

    y = yHist(end,:)';
    xNext.p = y(1:3);
    xNext.v = y(4:6);
    xNext.R = projectSO3(reshape(y(7:15), 3, 3));
    xNext.Omega = y(16:18);
end

function yDot = quadrotorOde(t, y, u, par)

    v = y(4:6);
    R = reshape(y(7:15), 3, 3);
    Omega = y(16:18);

    [a, OmegaDot] = rigidBodyRates(R, v, Omega, u, par, t);

    yDot = [v;
            a;
            reshape(R*hat(Omega), 9, 1);
            OmegaDot];
end

function xNext = stepModelLieRK4(x, u, par, t0)

    h = par.dt;

    Om1 = x.Omega;
    [a1, OmDot1] = rigidBodyRates(x.R, x.v, Om1, u, par, t0);

    v2 = x.v + 0.5*h*a1;
    R2 = x.R*expm(0.5*h*hat(Om1));
    Om2 = x.Omega + 0.5*h*OmDot1;
    [a2, OmDot2] = rigidBodyRates(R2, v2, Om2, u, par, t0 + 0.5*h);

    v3 = x.v + 0.5*h*a2;
    R3 = x.R*expm(0.5*h*hat(Om2));
    Om3 = x.Omega + 0.5*h*OmDot2;
    [a3, OmDot3] = rigidBodyRates(R3, v3, Om3, u, par, t0 + 0.5*h);

    v4 = x.v + h*a3;
    R4 = x.R*expm(h*hat(Om3));
    Om4 = x.Omega + h*OmDot3;
    [a4, OmDot4] = rigidBodyRates(R4, v4, Om4, u, par, t0 + h);

    OmegaBar = (Om1 + 2*Om2 + 2*Om3 + Om4)/6;

    xNext.p = x.p + h/6*(x.v + 2*v2 + 2*v3 + v4);
    xNext.v = x.v + h/6*(a1 + 2*a2 + 2*a3 + a4);
    xNext.R = x.R*expm(h*hat(OmegaBar));
    xNext.Omega = x.Omega + h/6*(OmDot1 + 2*OmDot2 + 2*OmDot3 + OmDot4);
end

function [a, OmegaDot] = rigidBodyRates(R, v, Omega, u, par, t)

    if nargin < 5
        t = 0;
    end

    [forceDist, momentDist] = disturbanceAtTime(t, par);
    fAeroWorld = R*sunAeroForceBody(R, v, par);

    a = par.g*par.e3 - u.T/par.m*R*par.e3 ...
        + (fAeroWorld + forceDist)/par.m;
    OmegaDot = par.J \ (u.tau + momentDist - cross(Omega, par.J*Omega));
end

function R = projectSO3(R)

    [U, ~, V] = svd(R);
    R = U*diag([1, 1, det(U*V')])*V';
end

%% ========================================================================
%% Result layer
function resultDir = saveMainRunResults(time, log, par, traj)

    if strlength(string(par.resultDir)) > 0
        resultDir = char(par.resultDir);
    else
        resultDir = fullfile(pwd, 'results', 'main');
    end

    if ~exist(resultDir, 'dir')
        mkdir(resultDir);
    end

    if par.saveMat
        par.resultDir = resultDir;
        save(fullfile(resultDir, 'main_run.mat'), ...
            'time', 'log', 'par', 'traj', '-v7.3');
    end

    fprintf('main results saved to: %s\n', resultDir);
end

%% SO(3) utility functions
function S = hat(w)

    S = [0, -w(3), w(2);
         w(3), 0, -w(1);
        -w(2), w(1), 0];
end

function v = vee(S)

    v = [S(3,2); S(1,3); S(2,1)];
end

function phi = LogSO3(R)

    % Robust SO(3) Log for aggressive flips:
    % The usual trace/acos formula,
    %   theta/(2*sin(theta))*vee(R - R'),
    % is sensitive near theta = pi and has the same acos derivative issue as
    % Tal Eq. (27). so3_log_map and manif both use the more robust path
    % matrix -> unit quaternion -> atan2 quaternion Log. The sign at exactly
    % pi is inherently ambiguous; qw >= 0 selects the principal branch.
    q = rotm2quat(projectSO3(R)).';
    phi = quatLogVectorWXYZ(q);
end

function Jl = leftJacobianSO3(phi)

    theta = norm(phi);
    Phi = hat(phi);

    if theta < 1e-6
        Jl = eye(3) + 0.5*Phi + (1/6)*Phi*Phi;
        return;
    end

    Jl = eye(3) ...
        + (1 - cos(theta))/theta^2*Phi ...
        + (theta - sin(theta))/theta^3*Phi*Phi;
end

function phi = quatLogVectorWXYZ(q)

    q = quatnormalize(q.').';
    if q(1) < 0
        q = -q;
    end

    v = q(2:4);
    nv = norm(v);
    qw = min(1, max(-1, q(1)));

    if nv < 1e-8
        nv2 = nv^2;
        scale = 2*(1 + nv2/6 + 3*nv2^2/40);
    else
        scale = 2*atan2(nv, qw)/nv;
    end

    phi = scale*v;
end

function st = initSecondOrderLPF(raw)

    raw = raw(:);
    st.x1 = raw;
    st.x2 = raw;
    st.y1 = raw;
    st.y2 = raw;
end

function [y, st] = secondOrderButterworthLPF(raw, st, h, cutoffHz)

    raw = raw(:);
    if cutoffHz <= 0 || h <= 0
        y = raw;
        st = initSecondOrderLPF(raw);
        return;
    end

    fs = 1/h;
    fc = min(cutoffHz, 0.45*fs);
    K = tan(pi*fc/fs);
    normFactor = 1/(1 + sqrt(2)*K + K^2);
    b0 = K^2*normFactor;
    b1 = 2*b0;
    b2 = b0;
    a1 = 2*(K^2 - 1)*normFactor;
    a2 = (1 - sqrt(2)*K + K^2)*normFactor;

    y = b0*raw + b1*st.x1 + b2*st.x2 - a1*st.y1 - a2*st.y2;

    st.x2 = st.x1;
    st.x1 = raw;
    st.y2 = st.y1;
    st.y1 = y;
end

function y = saturateVector(x, xmax)

    if isscalar(xmax)
        nx = norm(x);

        if nx > xmax
            y = x * xmax/nx;
        else
            y = x;
        end
    else
        y = min(max(x, -xmax), xmax);
    end
end
