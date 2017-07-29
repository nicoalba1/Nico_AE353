function hw1
clc
close all

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
params.tStop = 10;
% - Time step.
params.tStep = 1/25;

% DEFINE GEOMETRY

% Geometry of quadrotor
[geom.pRobot_in1,geom.fRobot]=GetRobotModel('quadmodel.mat');

% Geometry of room
dx = 3;
dy = 3;
dz = 1;
geom.pRoom_in0 =   [dx*[-1 1 1 -1 -1 1 1 -1];
                    dy*[-1 -1 1 1 -1 -1 1 1];
                    dz*[-1 -1 -1 -1 1 1 1 1]];
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
state.o_1in0 = zeros(3,1);

% Orientation of quadrotor (ZYX Euler Angles)
state.theta1 = 0;
state.theta2 = 0;
state.theta3 = 0;

end

function RunSimulation(params,geom,state)

% START-UP

% Create empty figure.
fig = [];

% Create data with an empty field for everything we want to save.
data = struct('t',[],'o_1in0',[],'theta1',[],'theta2',[],'theta3',[]);

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
    fs = 10;
    fig.text.label=text(0.05,0.95,'view: frame 0','fontweight','bold','fontsize',fs);
    fig.text.time=text(0.05,0.1,sprintf('t = %6.2f / %6.2f\n',state.t,params.tStop),'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.04,params.teamname,'fontsize',fs,'verticalalignment','top','fontweight','bold');
    
    % Create an axis for the view from frame 0.
    fig.view0.axis = axes('position',[0 0 1 1]);
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    axis equal;
    axis(1.25*[-3 3 -3 3 -1 1]);
    axis manual;
    hold on;
    axis off;
    view([90-37.5,20]);
    box on;
    set(gca,'projection','perspective');
    set(gca,'clipping','on','clippingstyle','3dbox');
    lighting gouraud
    fig.view0.light = light('position',[0;0;2],'style','local');
    
    fig.room = DrawMesh([],geom.pRoom_in0,geom.fRoom,[.9 .7 .9],0.1);
    fig.roomframe = DrawFrame([],geom.pRoomFrame_in0);
    
    % MODIFY: Add lines to draw the robot and the robot frame, similar to 
    %         the two lines just above. For the robot, I suggest alpha=0.6
    %         instead of alpha=0.1.
    % 
      fig.robot = DrawMesh([],geom.pRobot_in0,geom.fRobot,[.9 .7 .9],1);
      fig.robotframe = DrawFrame([],geom.pRobotFrame_in0);
    %
    
    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
else
    % UPDATE FIGURE
    
    set(fig.text.time,'string',sprintf('t = %6.2f / %6.2f\n',state.t,params.tStop));
    
    fig.room = DrawMesh(fig.room,geom.pRoom_in0,geom.fRoom);
    fig.roomframe = DrawFrame(fig.roomframe,geom.pRoomFrame_in0);
    
    % MODIFY: Add lines to draw the robot and the robot frame, similar to 
    %         the two lines just above.
    %
      fig.robot = DrawMesh(fig.robot,geom.pRobot_in0,geom.fRobot);
      fig.robotframe = DrawFrame(fig.robotframe,geom.pRobotFrame_in0);
    
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

% MODIFY: Add lines to compute geom.pRobot_in0 and geom.pRobotFrame_in0.
One = state.theta1;
Two = state.theta2;
Three = state.theta3;

o_1in0 = state.o_1in0;       % - copied for convenience
  
R_1in0 = [cos(One)*cos(Two),  cos(One)*sin(Two)*sin(Three)-sin(One)*cos(Three),  sin(One)*sin(Three)+cos(One)*sin(Two)*cos(Three);
          sin(One)*cos(Two),  sin(One)*sin(Two)*sin(Three)+cos(One)*cos(Three),  sin(One)*sin(Two)*cos(Three)-cos(One)*sin(Three);
             -sin(Two),                       cos(Two)*sin(Three),                                 cos(Two)*cos(Three)];

length(geom.pRobot_in1)
size(R_1in0*geom.pRobot_in1)
size(repmat(o_1in0,1,length(geom.pRobot_in1)))


geom.pRobot_in0 = R_1in0*geom.pRobot_in1 +repmat(o_1in0,1,length(geom.pRobot_in1));        % - coordinate transformation from geom.pRobot_in1
geom.pRobotFrame_in0 = R_1in0* geom.pRobotFrame_in1 +repmat(o_1in0,1,length(geom.pRobotFrame_in1));  % - coordinate transformation from geom.pRobotFrame_in1
%

end

function data = UpdateData(params,geom,state,data)
data.t(:,end+1) = state.t;
data.o_1in0(:,end+1) = state.o_1in0;
data.theta1(:,end+1) = state.theta1;
data.theta2(:,end+1) = state.theta2;
data.theta3(:,end+1) = state.theta3;
end

function state = UpdateState(params,geom,state)
x = [state.o_1in0; state.theta1; state.theta2; state.theta3];
[t,x] = ode45(@(t,x) GetXDot(t,x,params),[state.t state.t+params.tStep],x);
x = x(end,:)';
state.t = t(end);
state.o_1in0 = x(1:3);
state.theta1 = x(4);
state.theta2 = x(5);
state.theta3 = x(6);
end

function xdot = GetXDot(t,x,params)
o_1in0 = x(1:3);
theta1 = x(4);
theta2 = x(5);
theta3 = x(6);

% MODIFY: Add lines to define v_01in0 and w_01in1, and to compute
%         o_1in0dot and thetadot = [theta1dot; theta2dot; theta3dot].
%
  v_01in0 = exp(-t/4)* [sin(t); sin(2*t); sin(3*t)];    % - define linear velocity
  w_01in1 = 10*exp(-t)*[sin(t); sin(2*t); sin(3*t)];           % - define angular velocity


o_1in0dot = v_01in0;     % - replace to compute time derivative of position
thetadot = [0,        sin(theta3)/cos(theta2),              cos(theta3)/cos(theta2);
            0,            cos(theta3),                            -sin(theta3);
            1,    sin(theta3)*sin(theta2)/cos(theta2),   cos(theta3)*sin(theta2)/cos(theta2)] * w_01in1;
        

        

xdot = [o_1in0dot; thetadot];
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

