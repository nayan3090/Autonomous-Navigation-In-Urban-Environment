clear; clc;
%fclose(instrfindall)
load('rcnn.mat','rcnn')
load('rcnnStopSigns.mat','cifar10Net')
% ipA = '198.21.252.87';   portA = 9090;
% ipB = '198.21.223.147';  portB = 9091;
% udpA = udp(ipB,portB,'LocalPort',portA);
% fopen(udpA);
cam = webcam('USB2.0 PC CAMERA');
cam.Resolution = '160x120';
data=2;
while (1)   
    pic = snapshot(cam);
    [bboxes,score,label] = detect(rcnn,pic,'MiniBatchSize',128);
    [score, idx] = max(score);

    bbox = bboxes(idx, :);
    annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
    if label(idx) == 'stop'
        data=1;
    elseif label(idx) == 'school'
        data=0;
    else
        data=2;
    end
    disp(data)

    outputImage = insertObjectAnnotation(pic, 'rectangle', bbox, annotation);
%     fwrite(udpA,data)
    imagesc(outputImage)
end
%fclose(udpA)



