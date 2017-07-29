function hw4soln

% Setup the simulation.
[params,geom,state] = SetupSimulation();

% Run the simulation.
RunSimulation(params,geom,state);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS TO SETUP AND RUN THE SIMULATION
%

function [params,geom,state] = SetupSimulation()

% DEFINE PARAMETERS

% Parameters related to display.
% - Team name.
params.teamname = 'My Team (My Name and My Other Name)';

% Parameters related to data import/export.
% - File to record data.
params.data_filename = 'data.mat';
% - File to record an mp4 movie.
params.movie_filename = 'movie.mp4';
% - File to record a pdf snapshot.
params.snapshot_filename = 'snapshot.pdf';
% - Flag to say if you want to record a movie ("true" or "false").
params.makemovie = false;
% - Flag to say if you want to record a snapshot ("true" or "false").
params.makesnapshot = false;

% Parameters related to simulation.
% - State time.
params.tStart = 0;
% - Stop time.
params.tStop = 60;
% - Time step.
params.tStep = 1/25;

% Parameters related to physical properties of robot.
% - Acceleration of gravity.
params.g = 9.81;
% - Mass.
params.m = 0.4;
% - Spar length.
params.l = 0.5;
% - Principal moments of inertia.
params.J1 = 0.01;
params.J2 = 0.01;
params.J3 = 0.02;
% - Propellor coefficients.
params.kF = 1e-5;
params.kM = 1e-7;
% - Maximum squared spin rate of each propellor.
params.smax = (1e3)^2;
% - Location of two fiducial markers.
params.pMarker1_in1 = [0.5;0;0];
params.pMarker2_in1 = [0;0.5;0];

% Parameters related to trajectory tracking.
% - Maximum position error for loss of track.
params.max_o_error = 0.5;

% Parameters related to collision avoidance.
% - Start and goal position
params.o_start = [-3;0;3];
params.o_goal = [3;0;0];
% - Change these
params.krep = 1.0;
params.brep = 1.0;
params.katt = 100.0;
params.batt = 1.0;
% - Don't change these
params.maxvel = 0.5;
params.kdescent = 1e-2;
params.bdescent = params.maxvel*params.tStep;
params.rBoundingSphere = 0.81+0.5;

% Parameters related to physical properties of room
params.dx = 5;
params.dy = 5;
params.dz = 5;

% DEFINE GEOMETRY

% Geometry of quadrotor
[geom.pRobot_in1,geom.fRobot]=GetRobotModel('quadmodel.mat');

% Geometry of room
dx = 3;
dy = 3;
dz = 1;
geom.pRoom_in0 = [params.dx*[-1 1 1 -1 -1 1 1 -1];
                  params.dy*[-1 -1 1 1 -1 -1 1 1];
                  params.dz*[-1 -1 -1 -1 1 1 1 1]];
geom.fRoom = [1 2 3 4;
                5 6 7 8;
                1 2 6 5;
                2 3 7 6;
                3 4 8 7;
                4 1 5 8];

% Geometry of coordinate axes
pFrame = [0 1 0 0;
          0 0 1 0;
          0 0 0 1];
geom.pRoomFrame_in0 = pFrame;
geom.pRobotFrame_in1 = pFrame;

% DEFINE STATE

% Time
state.t = 0;

% Position of quadrotor
state.o_1in0 = params.o_start;

% Orientation of quadrotor (ZYX Euler Angles)
state.theta1 = 0.0;
state.theta2 = 0.0;
state.theta3 = 0.0;

% Linear velocity of quadrotor
state.v_01in0 = zeros(3,1);

% Angular velocity of quadrotor
state.w_01in1 = zeros(3,1);

% Input to quadrotor
state.u = zeros(4,1);

% Desired position
state.o_desired = params.o_start;

% Position error
state.o_error = 0;

% Time at which position error is exceeded
state.lostTrack = inf;

% Storage for controller
state.storage = [];

% Obstacles
% - Create a cell array to store obstacles
state.obst = {};
% - For all scenarios (#1, #2, #3, #4, #5)
state.obst = AddObstacle_HalfSpace(state.obst,[-params.dx;-params.dy;-params.dz],[1;0;0]);
state.obst = AddObstacle_HalfSpace(state.obst,[-params.dx;-params.dy;-params.dz],[0;1;0]);
state.obst = AddObstacle_HalfSpace(state.obst,[-params.dx;-params.dy;-params.dz],[0;0;1]);
state.obst = AddObstacle_HalfSpace(state.obst,[params.dx;params.dy;params.dz],[-1;0;0]);
state.obst = AddObstacle_HalfSpace(state.obst,[params.dx;params.dy;params.dz],[0;-1;0]);
state.obst = AddObstacle_HalfSpace(state.obst,[params.dx;params.dy;params.dz],[0;0;-1]);
% % - Uncomment for Scenario #2
% state.obst = AddObstacle_Sphere(state.obst,[0;1;0],1.25);
% % - Uncomment for scenario #3
% state.obst = AddObstacle_MovingSphere(state.obst,[-4.0;1.0;-12.0],0.15,[0.0;0.0;0.5]);
% state.obst = AddObstacle_MovingSphere(state.obst,[-3.0;-2.0;-9.0],0.15,[0.0;0.0;0.75]);
% state.obst = AddObstacle_MovingSphere(state.obst,[-2.0;-2.0;-7.0],0.15,[0.0;0.0;0.85]);
% state.obst = AddObstacle_MovingSphere(state.obst,[-1.0;4.0;-10.0],0.15,[0.0;0.0;0.6]);
% state.obst = AddObstacle_MovingSphere(state.obst,[0.0;-3.0;-11.0],0.15,[0.0;0.0;0.9]);
% state.obst = AddObstacle_MovingSphere(state.obst,[1.0;-1.0;-7.0],0.15,[0.0;0.0;1.1]);
% state.obst = AddObstacle_MovingSphere(state.obst,[2.0;0.5;-9.0],0.15,[0.0;0.0;0.5]);
% state.obst = AddObstacle_MovingSphere(state.obst,[3.0;-4.0;-9.0],0.15,[0.0;0.0;1.5]);
% state.obst = AddObstacle_MovingSphere(state.obst,[4.0;3.0;-9.0],0.15,[0.0;0.0;0.95]);
% state.obst = AddObstacle_MovingSphere(state.obst,[3.0;-2.0;-10.0],0.15,[0.0;0.0;0.8]);
% state.obst = AddObstacle_Sphere(state.obst,[0;1;3],1.7);
% state.obst = AddObstacle_Sphere(state.obst,[0;1;1],1.3);
% state.obst = AddObstacle_Sphere(state.obst,[0;1;-0.5],0.9);
% state.obst = AddObstacle_Sphere(state.obst,[0;2.2;0.5],0.4);
% state.obst = AddObstacle_Sphere(state.obst,[0;2.6;0.3],0.3);
% state.obst = AddObstacle_Sphere(state.obst,[0;3.0;0.1],0.2);
% state.obst = AddObstacle_Sphere(state.obst,[0;-0.3;0.5],0.4);
% state.obst = AddObstacle_Sphere(state.obst,[0;-0.7;0.3],0.3);
% state.obst = AddObstacle_Sphere(state.obst,[0;-1.1;0.1],0.2);
% % - Uncomment for scenario #4
% state.obst = AddRandomSpheres(25,0.15,0.85,state.o_desired,params.o_goal,state.obst,params);
% - Uncomment for scenario #5
state.obst = AddObstacle_Sphere(state.obst,[0;1.5;0.0],0.75);
state.obst = AddObstacle_Sphere(state.obst,[0;-1.5;0.0],0.75);
state.obst = AddObstacle_Sphere(state.obst,[1.0;0;2.0],0.75);



end

function RunSimulation(params,geom,state)

% START-UP

% Create empty figure.
fig = [];

% Create data with an empty field for everything we want to save.
data = struct('t',[],...
              'o_1in0',[],...
              'theta1',[],'theta2',[],'theta3',[],...
              'v_01in0',[],...
              'w_01in1',[],...
              'u',[],...
              'o_desired',[]);

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (params.makemovie)
    myV = VideoWriter(params.movie_filename,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/params.tStep;
    open(myV);
end

% LOOP

% Loop until break.
while (1)
    
    % Update geometry.
    geom = UpdateGeometry(params,geom,state);
    
    % Update figure (create one if fig is empty).
    fig = UpdateFigure(params,geom,state,fig);
    
    % Update data.
    data = UpdateData(params,geom,state,data);
    
    % If making a movie, store the current figure as a frame.
    if (params.makemovie)
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
    % Stop if time has reached its maximum.
    if ((state.t>=params.tStop)||done)
        break;
    end
    
    % Update state (integrate equations of motion).
    state = UpdateState(params,geom,state);
    
end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (params.makemovie)
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
save(params.data_filename,'-struct','data');

% Save the snapshot, if necessary.
if (params.makesnapshot)
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',params.snapshot_filename);
end

end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS RELATED TO DRAWING
%

function fig = UpdateFigure(params,geom,state,fig)
if (isempty(fig))
    % CREATE FIGURE
    
    % Clear the current figure.
    clf;
    
    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fs = 14;
    fig.text.status=text(0.05,0.975,sprintf('position error: %5.2f',state.o_error),'fontweight','bold','fontsize',fs,'color','k','verticalalignment','top');
    fig.text.time=text(0.05,0.12,sprintf('t = %6.2f / %6.2f\n',state.t,params.tStop),'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.06,params.teamname,'fontsize',fs,'verticalalignment','top','fontweight','bold');
    
    % Create an axis for the view from frame 0.
    fig.view0.axis = axes('position',[0 0 1 1]);
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    axis equal;
    axis(1.1*[-params.dx params.dx -params.dy params.dy -params.dz params.dz]);
    axis manual;
    hold on;
    axis off;
    view([270-37.5,-20]);
    camroll(180);
    box on;
    set(gca,'projection','perspective');
    set(gca,'clipping','on','clippingstyle','3dbox');
    lighting gouraud
    fig.view0.light = light('position',[0;0;-2],'style','local');
    
    fig.room = DrawMesh([],geom.pRoom_in0,geom.fRoom,[.9 .7 .9],0.1);
    fig.roomframe = DrawFrame([],geom.pRoomFrame_in0);
    fig.robot = DrawMesh([],geom.pRobot_in0,geom.fRobot,'y',0.6);
    fig.robotframe = DrawFrame([],geom.pRobotFrame_in0);
    
    fig.o_desired = DrawTrace([],nan(3,1),'k',10/params.tStep);
    fig.o = DrawTrace([],state.o_1in0,[1,0.6,0],10/params.tStep);
    
    % Draw obstacles
    % - Create unit sphere
    [fig.xSphere,fig.ySphere,fig.zSphere]=sphere(16);
    [m,n]=size(fig.xSphere);
    % - Create template array for colors
    c = ones(m,n,3);
    % - Draw spheres
    for i=1:length(state.obst)
        if (state.obst{i}.type == 1)
            c(:,:,1) = 0.75;
            c(:,:,2) = 0.25;
            c(:,:,3) = 0.25;
            fig.obst(i) = DrawBubble([],state.obst{i}.p,state.obst{i}.s,fig.xSphere,fig.ySphere,fig.zSphere,c,0.8);
        end
    end
    fig.c = c;
    
    % Draw bubble at desired position
    c(:,:,1) = 0.25;
    c(:,:,2) = 0.25;
    c(:,:,3) = 0.75;
    fig.robotbubble = DrawGlowingBubble([],state.o_desired,params.rBoundingSphere,fig.xSphere,fig.ySphere,fig.zSphere,c,0.4);
    % Draw bubble at goal position
    c(:,:,1) = 0.25;
    c(:,:,2) = 0.75;
    c(:,:,3) = 0.25;
    fig.goalbubble = DrawBubble([],params.o_goal,params.rBoundingSphere,fig.xSphere,fig.ySphere,fig.zSphere,c,0.2);
    
    % Make everything look pretty
    h = findobj('Type','surface');
    set(h,'FaceLighting','gouraud',...
          'FaceColor','interp',...
          'EdgeColor',[.4 .4 .4],...
          'LineStyle','none',...
          'BackFaceLighting','lit',...
          'AmbientStrength',0.4,...
          'DiffuseStrength',0.6,...
          'SpecularStrength',0.5);
    material default
    
    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
else
    % UPDATE FIGURE
    
    set(fig.text.time,'string',sprintf('t = %6.2f / %6.2f\n',state.t,params.tStop));
    if (~isinf(state.lostTrack))
        set(fig.text.status,'string',sprintf('position error: %5.2f\nlost track at time: %5.2f',state.o_error,state.lostTrack),'color','r');
    else
        set(fig.text.status,'string',sprintf('position error: %5.2f',state.o_error));
    end
    
    fig.room = DrawMesh(fig.room,geom.pRoom_in0,geom.fRoom);
    fig.roomframe = DrawFrame(fig.roomframe,geom.pRoomFrame_in0);
    fig.robot = DrawMesh(fig.robot,geom.pRobot_in0,geom.fRobot);
    fig.robotframe = DrawFrame(fig.robotframe,geom.pRobotFrame_in0);
    
    fig.o_desired = DrawTrace(fig.o_desired,state.o_desired);
    fig.o = DrawTrace(fig.o,state.o_1in0);
    
    % Draw obstacles
    for i=1:length(state.obst)
        if (state.obst{i}.type == 1)
            if (any(state.obst{i}.v))
                fig.obst(i) = DrawBubble(fig.obst(i),state.obst{i}.p,state.obst{i}.s,fig.xSphere,fig.ySphere,fig.zSphere);
            end
        end
    end
    % Draw bubble at desired position
    fig.robotbubble = DrawGlowingBubble(fig.robotbubble,state.o_desired,params.rBoundingSphere,fig.xSphere,fig.ySphere,fig.zSphere);
    
    
%     fig.c(:,:,1) = 0.75;
%     fig.c(:,:,2) = 0.15;
%     fig.c(:,:,3) = 0.5;
%     set(fig.goalbubble,'cdata',fig.c);
    
end
drawnow;
end

function [p,f]=GetRobotModel(filename)
load(filename);
end

function onkeypress(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

function bubble = DrawBubble(bubble,o,r,x,y,z,c,a)
if (isempty(bubble))
    bubble = surf(o(1)+r*x,o(2)+r*y,o(3)+r*z,c,'FaceAlpha',a);
else
    set(bubble,'xdata',o(1)+r*x,'ydata',o(2)+r*y,'zdata',o(3)+r*z);
end
end

function bubble = DrawGlowingBubble(bubble,o,r,x,y,z,c,a)
if (isempty(bubble))
    bubble.surface = surf(o(1)+r*x,o(2)+r*y,o(3)+r*z,c,'FaceAlpha',a);
    bubble.light = light('position',o,'style','local');
else
    set(bubble.surface,'xdata',o(1)+r*x,'ydata',o(2)+r*y,'zdata',o(3)+r*z);
    set(bubble.light,'position',o);
end
end


function trace = DrawTrace(trace,p,c,n)
if (isempty(trace))
    trace = line(p(1),p(2),p(3),'color',c,'marker','.','linestyle','none','UserData',n);
else
    x = get(trace,'xdata');
    y = get(trace,'ydata');
    z = get(trace,'zdata');
    n = get(trace,'UserData');
    if (length(x)>=n)
        x = x(2:end);
        y = y(2:end);
        z = z(2:end);
    end
    x(:,end+1) = p(1);
    y(:,end+1) = p(2);
    z(:,end+1) = p(3);
    set(trace,'xdata',x,'ydata',y,'zdata',z);
end
end

function mesh = DrawMesh(mesh,p,f,color,alpha)
if isempty(mesh)
    mesh = patch('Vertices',p','Faces',f,...
                 'FaceColor',color,'FaceAlpha',alpha,'EdgeAlpha',alpha);
else
    set(mesh,'vertices',p');
end
end

function frame = DrawFrame(frame,p)
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS RELATED TO ANALYSIS
%

function geom = UpdateGeometry(params,geom,state)

o_1in0 = state.o_1in0;
R_1in0 = R_ZYX(state.theta1,state.theta2,state.theta3);
geom.pRobotFrame_in0 = Transform(o_1in0,R_1in0,geom.pRobotFrame_in1);
geom.pRobot_in0 = Transform(o_1in0,R_1in0,geom.pRobot_in1);

end

function data = UpdateData(params,geom,state,data)
data.t(:,end+1) = state.t;
data.o_1in0(:,end+1) = state.o_1in0;
data.theta1(:,end+1) = state.theta1;
data.theta2(:,end+1) = state.theta2;
data.theta3(:,end+1) = state.theta3;
data.v_01in0(:,end+1) = state.v_01in0;
data.w_01in1(:,end+1) = state.w_01in1;
data.u(:,end+1) = state.u;
data.o_desired(:,end+1) = state.o_desired;
end

function [u,storage] = GetControlInput(y,o_desired,params,storage)
if (isempty(storage))
    fprintf(1,'Initializing controller...\n');
    
    % initialize input and state estimate
    storage.xhat_d = zeros(12,1);

    % do control design
    % - Controller
    Av = [0 -params.g 0; 0 0 params.g; 0 0 0];
    Bv = [zeros(3) [0;0;-1/params.m]];
    Atheta = [0 0 1; 0 1 0; 1 0 0];
    Bw = [diag([1/params.J1,1/params.J2,1/params.J3]) zeros(3,1)];
    Ac = [zeros(3) zeros(3) eye(3) zeros(3);
          zeros(3) zeros(3) zeros(3) Atheta;
          zeros(3) Av zeros(3) zeros(3);
          zeros(3) zeros(3) zeros(3) zeros(3)];
    Bc = [zeros(3,4);
          zeros(3,4);
          Bv;
          Bw];
    storage.Ad = eye(size(Ac))+params.tStep*Ac;
    storage.Bd = params.tStep*Bc;
    Qd = eye(12);
    Rd = eye(4);
    storage.Kd = myDLQR(storage.Ad,storage.Bd,Qd,Rd);
    % - Observer
    storage.Cd = [eye(3,3) [-params.pMarker1_in1(2) params.pMarker1_in1(3) 0; params.pMarker1_in1(1) 0 -params.pMarker1_in1(3); 0 -params.pMarker1_in1(1) params.pMarker1_in1(2)] zeros(3,3) zeros(3,3);
                  eye(3,3) [-params.pMarker2_in1(2) params.pMarker2_in1(3) 0; params.pMarker2_in1(1) 0 -params.pMarker2_in1(3); 0 -params.pMarker2_in1(1) params.pMarker2_in1(2)] zeros(3,3) zeros(3,3)];
    Qd = eye(6);
    Rd = eye(12);
    storage.Ld = myDLQR(storage.Ad',storage.Cd',inv(Rd),inv(Qd))';
end

% Nominal input
ubar = [0;0;0;params.m*params.g];

% Controller
u = ubar-storage.Kd*storage.xhat_d;
[u,s] = GetBoundedInputs(u,params);
u_d = u-ubar;

% Observer
% - ybar
ybar = [o_desired + params.pMarker1_in1; o_desired + params.pMarker2_in1];
% - y_d
y_d = y-ybar;
% - xhat
storage.xhat_d = storage.Ad*storage.xhat_d + storage.Bd*u_d - storage.Ld*(storage.Cd*storage.xhat_d-y_d);


end

function state = UpdateState(params,geom,state)
x = [state.o_1in0;
     state.theta1;
     state.theta2;
     state.theta3;
     state.v_01in0;
     state.w_01in1];

% Update position of moving obstacles
for i=1:length(state.obst)
    if (state.obst{i}.type==1)
        state.obst{i}.p = state.obst{i}.p+state.obst{i}.v*params.tStep;
    end
end
 
% Output
R_1in0 = R_ZYX(state.theta1,state.theta2,state.theta3);
y = [state.o_1in0+R_1in0*params.pMarker1_in1; state.o_1in0+R_1in0*params.pMarker2_in1];

% Position error
state.o_error = norm(state.o_desired - state.o_1in0);
if (state.o_error > params.max_o_error)
    if (isinf(state.lostTrack))
        state.lostTrack = state.t;
    end
end

% Gradient descent to steer desired position to goal position
state.o_desired = GradientDescent(state.o_desired,params.o_goal,state.obst,params);

% Input
[u,state.storage] = GetControlInput(y,state.o_desired,params,state.storage);

[t,x] = ode45(@(t,x) GetXDot(t,x,u,params),[state.t state.t+params.tStep],x);
x = x(end,:)';
state.t = t(end);
state.o_1in0 = x(1:3);
state.theta1 = x(4);
state.theta2 = x(5);
state.theta3 = x(6);
state.v_01in0 = x(7:9);
state.w_01in1 = x(10:12);
state.u = u;
end

function xdot = GetXDot(t,x,u,params)
o_1in0 = x(1:3);
theta1 = x(4);
theta2 = x(5);
theta3 = x(6);
v_01in0 = x(7:9);
w_01in1 = x(10:12);

R_1in0 = R_ZYX(theta1,theta2,theta3);

o_1in0dot = v_01in0;
thetadot = GetAngularRates_ZYX(theta1,theta2,theta3,w_01in1);

f_in0 = [0;0;params.m*params.g] - R_1in0*[0;0;u(4,1)];
tau_in1 = u(1:3,1);
J_in1 = diag([params.J1,params.J2,params.J3]);

v_01in0dot = (1/params.m)*f_in0;
w_01in1dot = inv(J_in1)*(tau_in1-wedge(w_01in1)*J_in1*w_01in1);

xdot = [o_1in0dot; thetadot; v_01in0dot; w_01in1dot];
end


function [u,s] = GetBoundedInputs(ubar,params)

W = [params.kF*params.l -params.kF*params.l 0 0;
     0 0 params.kF*params.l -params.kF*params.l;
     params.kM params.kM -params.kM -params.kM;
     params.kF params.kF params.kF params.kF];

sbar = W\ubar;

for i=1:4
    if (sbar(i)<0)
        s(i,1)=0;
    elseif (sbar(i)>params.smax)
        s(i,1)=params.smax;
    else
        s(i,1)=sbar(i);
    end
end

u = W*s;

end

function R = RX(h)
R = [1 0 0;
     0 cos(h) -sin(h);
     0 sin(h) cos(h)];
end
 
function R = RY(h)
R = [cos(h) 0 sin(h);
     0 1 0;
     -sin(h) 0 cos(h)];
end
 
function R = RZ(h)
R = [cos(h) -sin(h) 0;
     sin(h) cos(h) 0;
     0 0 1];
end
 
function R = R_ZYX(theta1,theta2,theta3)
R = RZ(theta1)*RY(theta2)*RX(theta3);
end

function thetadot = GetAngularRates_ZYX(theta1,theta2,theta3,w)
c2 = cos(theta2);
s2 = sin(theta2);
c3 = cos(theta3);
s3 = sin(theta3);
A = [   -s2     0       1;
        c2*s3   c3      0;
        c2*c3   -s3     0];
thetadot = A\w;
end

function wHat = wedge(w)
wHat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end

function p_inj = Transform(o_kinj,R_kinj,p_ink)
p_inj = zeros(size(p_ink));
for i=1:size(p_ink,2)
    p_inj(:,i) = o_kinj + R_kinj*p_ink(:,i);
end
end

function Kd = myDLQR(Ad,Bd,Qd,Rd,tol)
if (nargin<5)
    tol = 1e-6;
end
[nStates,nInputs] = size(Bd);
Pplus = eye(nStates);
Kdplus = inf(nInputs,nStates);
i = 0;
while (1)
    P = Qd + Ad'*Pplus*Ad - Ad'*Pplus*Bd*inv(Rd+Bd'*Pplus*Bd)*Bd'*Pplus*Ad;
    Kd = inv(Rd+Bd'*Pplus*Bd)*Bd'*Pplus*Ad;
    % Check to see if P is still positive definite
    % - Find eigenvalues of P
    s = eig(P);
    % - These eigenvalues will be real, because P will be symmetric, that
    %   is guaranteed. But we still need to check that they are positive.
    if any(s<=0)
        error('P is no longer positive definite in myDLQR');
    end
    % Check to see if Kd has converged
    if all(abs(Kd-Kdplus)<tol)
        % - It has converged, so return it.
        return
    else
        % - It has not converged, so update Pplus and Kdplus
        Pplus = P;
        Kdplus = Kd;
    end
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS RELATED TO COLLISION AVOIDANCE
%

function obst = AddRandomSpheres(n,smin,smax,o_start,o_goal,obst,params)
% n = number of spheres to add
% smin = minimum radius
% smax = maximum radius
% o_start = start position
% o_goal = goal position

% Loop to add n spheres one at a time
for i=1:n
    
    % Keep trying until we add a sphere that does not interfere
    % with the start or goal position of the quadrotor.
    while(1)
        % Random center position in the room
        p = 0.8*[params.dx;params.dy;params.dz].*(-1+2*rand(3,1));
        % Random radius
        s = smin+(smax-smin)*rand;
        % Check for non-interference
        [dd,tmp] = SphereSphere(o_start,params.rBoundingSphere,p,s);
        [dg,tmp] = SphereSphere(o_goal,params.rBoundingSphere,p,s);
        if ((dd>0)&(dg>1))
            obst = AddObstacle_Sphere(obst,p,s);
            break;
        end
    end
    
end
end

function obst = AddObstacle_Sphere(obst,p,s)
% Code that says this obstacle is a sphere
onew.type = 1;
% Position and radius
onew.p = p;
onew.s = s;
% Velocity vector
onew.v = [0;0;0];
% Add obstacle to end of cell array
obst{end+1}=onew;
end

function obst = AddObstacle_MovingSphere(obst,p,s,v)
% Code that says this obstacle is a sphere
onew.type = 1;
% Position and radius
onew.p = p;
onew.s = s;
% Velocity vector
onew.v = v;
% Add obstacle to end of cell array
obst{end+1}=onew;
end

function obst = AddObstacle_HalfSpace(obst,p,n)
% Code that says this obstacle is a half-space
onew.type = 3;
% Position and normal vector
onew.p = p;
onew.n = n;
% Add obstacle to end of cell array
obst{end+1}=onew;
end

function [d,dgrad] = SphereSphere(q,r,p,s)
v = q-p;
vnorm = sqrt(v'*v);
d = vnorm-(r+s);
dgrad = v/vnorm;
end

function [d,dgrad] = SphereHalfSpace(q,r,p,n)
v = q-p;
vnorm = sqrt(v'*v);
d = (n'*v)-r; % assumes a "front" and "back" to the plane...
dgrad = n;
end

function gradfatt = GetAttractiveGradient(q,q_goal,params)
v = q-q_goal;
vnorm = sqrt(v'*v);
if (vnorm <= params.batt)
    gradfatt = params.katt*v;
else
    gradfatt = params.katt*params.batt*(v/vnorm);
end
end

function gradfrep = GetRepulsiveGradient(q,obst,params)
gradfrep = zeros(size(q));
for i=1:length(obst)
    
    if (obst{i}.type == 1)
        [d,dgrad] = SphereSphere(q,params.rBoundingSphere,obst{i}.p,obst{i}.s);
    elseif (obst{i}.type == 3)
        [d,dgrad] = SphereHalfSpace(q,params.rBoundingSphere,obst{i}.p,obst{i}.n);
    else
        error('bad obst{i}.type in GetRepulsiveGradient');
    end
    
    if (d <= params.brep)
        gradfrep = gradfrep + (params.krep*((1/params.brep)-(1/d))*(1/(d^2))*dgrad);
    end
    
end
end

function gradf = GetGradient(q,q_goal,obst,params)
% YOUR CODE HERE (PROBLEM 5a)
gradf = GetAttractiveGradient(q,q_goal,params)+GetRepulsiveGradient(q,obst,params);
end

function q=GradientDescent(q,q_goal,obst,params)
% - Get gradient of potential function
gradf = GetGradient(q,q_goal,obst,params);
% - Do gradient descent (i.e., take a step)
dq = -params.tStep*(params.kdescent*gradf);
dqnorm = norm(dq);
if (dqnorm > params.bdescent)
    dq = params.bdescent*(dq/dqnorm);
end
q = q+dq;
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
















