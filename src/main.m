clear
clc
fclose(instrfindall)

a = arduino('COM3','Uno','Libraries','Servo');
v = servo(a,'D13','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
s = servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);

%% Define computer-specific variables
% Modify these values to be those of your first computer:
ipA = '198.21.252.87'; portA = 9090;
% Modify these values to be those of your second computer:
ipB = '198.21.248.165'; portB = 9091;
%% Create UDP Object
udpB = udp(ipA,portA,'LocalPort',portB);
%% Connect to UDP Object
fopen(udpB);
%%
load('calibrationSession_final')
cam = webcam('USB_Camera');
cam.resolution = '1280X720';

%% Initialization of Kalman filter
draw_lx=[0;0];
draw_rx=[0;0];
Pl=[1 0;0 1];
Pr=[1 0;0 1];
Q = [0.19018771 0; 0 0.073000203];
RR = [0.00000001 0;0 0.00000001];

%% Main code
state = 1;
while (1)
    pic = snapshot(cam);
    pic = imresize(pic,0.5);
    shape = size(pic);
    gray_pic = rgb2gray(pic); % Gray picture
    edge_pic = edge(gray_pic,'canny',[0.2,0.36]); % Edge detection
    %% Cut the picture
    a = [shape(2)*0.4,shape(2)*0.6,shape(2),0];
    b = [shape(1)*0.5, shape(1)*0.5,0.9*shape(1),0.9*shape(1)];
    bw = roipoly(pic,a,b);
    BW = (edge_pic(:,:,1)&bw);
    %% Hough transform to detect lines
    [H,T,R] = hough(BW);
    P = houghpeaks(H,8,'Threshold',ceil(0.3*max(H(:))));
    lines = houghlines(BW,T,R,P,'FillGap',6,'MinLength',2);
    anglethres=0.01; %separate left/right by orientation threshold
    leftlines=[];
    rightlines=[]; %Two group of lines
    imagesc(BW);
    hold on
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        x1=lines(k).point1(1);
        y1=lines(k).point1(2);
        x2=lines(k).point2(1);
        y2=lines(k).point2(2);
        if (x2>=shape(2)/2) && ((y2-y1)/(x2-x1)>anglethres)
            rightlines=[rightlines;x1,y1;x2,y2];
        elseif (x2<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*anglethres)) 
            leftlines=[leftlines;x1,y1;x2,y2];       
        end
    end
    
    %% Post processing
    draw_y=[shape(1)*0.6,shape(1)]; %two row coordinates
    if isempty(leftlines)
        PL=0;
    else
       PL=polyfit(leftlines(:,2),leftlines(:,1),1);
    end
    if isempty(rightlines)
        PR=0;
    else
    PR=polyfit(rightlines(:,2),rightlines(:,1),1);
    end
    
    % Kalman filter left point
    L_p=draw_lx';
    P_pl=Pl + Q;
    Kl=P_pl*((P_pl+RR)^(-1));
     draw_lx=polyval(PL,draw_y); %two col coordinates of left line
    L_m = draw_lx;
    draw_lx = L_p+Kl*(L_m'-L_p);
    Pl = (1-Kl)*P_pl;
    
    % Kalman filter for right line
    R_p=draw_rx';
    P_pr=Pr + Q;
    Kr=P_pr*((P_pr+RR)^(-1));
    draw_rx=polyval(PR,draw_y); %two col coordinates of right line
    R_m = draw_rx;
    draw_rx = R_p+Kr*(R_m'-R_p);
    Pr = (1-Kr)*P_pr;
    
    %% Inverse projection from image frame to vehicle frame
    Far_point_l = pointsToWorld(calibrationSession.CameraParameters,calibrationSession.CameraParameters.RotationMatrices(:,:,1),calibrationSession.CameraParameters.TranslationVectors(1,:),2*[draw_rx(1,1),draw_y(1,1)]);
    Far_point_r = pointsToWorld(calibrationSession.CameraParameters,calibrationSession.CameraParameters.RotationMatrices(:,:,1),calibrationSession.CameraParameters.TranslationVectors(1,:),2*[draw_rx(1,1),draw_y(1,1)]);
    Near_point_l = pointsToWorld(calibrationSession.CameraParameters,calibrationSession.CameraParameters.RotationMatrices(:,:,1),calibrationSession.CameraParameters.TranslationVectors(1,:),2*[draw_lx(2,1),draw_y(1,2)]);
    Near_point_r = pointsToWorld(calibrationSession.CameraParameters,calibrationSession.CameraParameters.RotationMatrices(:,:,1),calibrationSession.CameraParameters.TranslationVectors(1,:),2*[draw_rx(2,1),draw_y(1,2)]);
    
    Fl_v = [Far_point_l(1)-100,385-Far_point_l(2)];
    Fr_v = [Far_point_r(1)-100,385-Far_point_r(2)];
    Nl_v = [Near_point_l(1)-100,385-Near_point_l(2)];
    Nr_v = [Near_point_r(1)-100,385-Near_point_r(2)];
    
    Xcf = 1/2 * (Fl_v(1) + Fr_v(1));
    Ycf = 1/2 * (Fl_v(2) + Fr_v(2));
    Xcn = 1/2 * (Nl_v(1) + Nr_v(1));
    Ycn = 1/2 * (Nl_v(2) + Nr_v(2));
    
    %% Stanley control
    theta_e = -((-1) * (atan((Ycf - Ycn)/(Xcf - Xcn)))*180/pi - 90+145);
    efa = -(Xcn + ((Xcf-Xcn)/(Ycf-Ycn))*(Ycn - 100)-17);
    
    k1 = 0.003;
    k2 = 0.002;
    phi = -k1*theta_e - k2*efa+0.5;
    
    draw_mid=draw_lx + (draw_rx-draw_lx)/2; % mid-line
       
    imagesc(pic);
    hold on;
    plot(draw_lx,draw_y,'LineWidth',2,'Color','red');
    hold on
    plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
    hold on
    plot(draw_mid,draw_y,'LineWidth',2,'Color','red');
    hold off
    
     if phi >= 0.75
            phi = 0.75;
     elseif phi <= 0.25
            phi = 0.25;
     else
            phi = phi;
     end
    
     if isempty(leftlines) && theta_e>0
            phi = 0.3;
     elseif isempty(leftlines) && theta_e<0
            phi=0.6;
     else
            phi=phi;
     end
    
     if isempty(rightlines) && theta_e>0
            phi=0.3;
     elseif isempty(rightlines) && theta_e<0
            phi=0.6;
     else
            phi=phi;
     end
    
     if isempty(leftlines) && isempty(rightlines)
            phi=0.5;
     else
            phi=phi;
     end
     writePosition(s,phi)
    %% Connecting two laptops  
    if udpB.BytesAvailable > 0
        data = fread(udpB, udpB.BytesAvailable);
        flushinput(udpB);
    end
    
    %% Write speed to different scenarios
    if data == 0      
        state = 0;
    elseif  data == 1
        state = 1;
    end
    if state == 0     % From school sign to stop sign
        writePosition(v,0.545)
        pause(0.1)        
        
        writePosition(v,0.49)
        pause(0.15)
        
    elseif state == 1 && data == 1    % From stop sign to start
        writePosition(v,0.5)
        pause(3)
        
        writePosition(v,0.6)
        pause(0.3)
        writePosition(v,0.49)
        pause(0.15)
        
    elseif state == 1 && data == 2    % From stop sign to school sign
        writePosition(v,0.55)
        pause(0.1)        
        
        writePosition(v,0.49)
        pause(0.15) 
    end
    disp(state)
end
