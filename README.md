# Actividad-6-SLAM-de-LiDAR-


Se establecen las condiciones iniciales tanto de nuestro vehiculo como de la simulacion y el tiempo.
  ``` matlab
%% Simulation setup
% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:130;        % Time array

% Initial conditions
initPose = [1;2;90];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;
  ``` 
  Se carga el mapa y se configura el sensor LIDAR aunado a esto se crea el visulizador
  
  ``` matlab
% Load map
close all
load exampleMap

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi,pi,250);
lidar.maxRange = 1.5;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);
  ``` 
  
  
  Se definen las 4 trayectorias dependiendo la que se quiera realizar.
  ``` matlab
%% Path planning and following

%Primera trayectoria
waypoints = [initPose(1:2)'; 9 2; 9 6];

%Segunda trayectoria
%waypoints = [initPose(1:2)'; 2 6; 2 2; 9 8; 7 6; 9 3; 7 2];

%Tercera trayectoria
%waypoints = [initPose(1:2)'; 1 4; 1 3; 1 2; 1 1; 2 1; 2 2; 2 3; 2 4; 3 4; 3 3; 3 2; 3 1; 4 3; 4 4];

%Cuarta trayectoria
%waypoints = [initPose(1:2)'; 2 10; 11 8; 8 2; 8 8; 1 2];
``` 

Se establece el controlador dependiendo de las necesidades del proyecto.

  ``` matlab
% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.60;
controller.DesiredLinearVelocity = 0.35;
controller.MaxAngularVelocity = 50;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [5 10];
vfh.RobotRadius = L;
vfh.SafetyDistance = L;
vfh.MinTurningRadius = 0.25;
  ``` 
Se establecen los parametros e instrucciones para hacer posbile la simulacion.
  ``` matlab
%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
end
  ``` 

 # Resultados
  ## Trayectoria 1 
  ![image](https://github.com/Bere901/Actividad-6-SLAM-de-LiDAR-/assets/99983026/55810ef1-d5f1-493b-8dca-f3e17cdae424)
  ## Trayectoria 2
  ![image](https://github.com/Bere901/Actividad-6-SLAM-de-LiDAR-/assets/99983026/0c7b67f5-9830-4c87-b129-19feb8c17b74)
  ## Trayectoria 3
  ![image](https://github.com/Bere901/Actividad-6-SLAM-de-LiDAR-/assets/99983026/a36a9b56-1cab-4b8d-8b6b-86ba89f76d8a)
  ## Trayectoria 4
  ![image](https://github.com/Bere901/Actividad-6-SLAM-de-LiDAR-/assets/99983026/87a5d5f1-7626-4749-b469-8d3b9009f16f)


