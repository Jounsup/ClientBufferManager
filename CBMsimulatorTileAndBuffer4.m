% Client Buffer Manager simulation - Sever, Network and Client buffer
clear all; close all; clc;

worldTime = 0; % global time counter
playOut = 0; % timing for play out
prefetch = 1; % number of segments to prefetch
numFrame = 300;

%% Server - video chunks manifest
%r = [3 6 10 16 27]*1e6;
r = [3.9 6 13 27 42.7]*1e6;
load('CUwDepth.mat');
% tic
% load('AssetsNew.mat');
% toc
dd = 5; W = 5;
bufGof = W*7;
tileCount = tileCountWdepth(1,dd);
Umatrix = zeros(tileCount, length(r));
Cmatrix = zeros(tileCount, length(r));
Pmatrix = zeros(tileCount, bufGof);
bufState = zeros(tileCount, numFrame);
stateMatrix = 0;
inip = [zeros(1,floor(3*tileCount/8)) ones(1,floor(tileCount/4)) zeros(1,floor(3*tileCount/8))];
for tt=1:tileCount
    Umatrix(tt,:) = UwDepth(1,:,dd);
    Cmatrix(tt,:) = r/sum(inip);%CwDepth(1,:,dd);
end
p = inip; 
grd = 1;
for bb=1:1%bufGof
%     if mod(bb,5)==0, grd = grd+1; end
%     Pmatrix(:,bb) = p/sum(p)/grd;
%     p = conv(p,ones(1,3),'same'); p = p/sum(p);
    Perr = 0.1+0.3*bb/bufGof;
    Pmatrix(:,bb) = p*(1-Perr)+(1-p)*Perr;
end
%figure(3); bar3(Pmatrix);
for bb=1:bufGof
    for rr=1:length(r)
        U((bb-1)*tileCount+1:bb*tileCount,rr) = Umatrix(:,rr).*Pmatrix(:,bb);
    end
    C((bb-1)*tileCount+1:bb*tileCount,:) = Cmatrix;
end
FrameToWorldScale = 0;
FrameToWorldRotation = 0;
FrameToWorldTranslation = 0;
numberofSegment = 1000; % number of segments for simulation
segmentDuration = 4/30; % chunk duration (sec)
startTime = 0;
duration = numberofSegment * segmentDuration;
endTime = startTime + duration;
startSegment = 0;
degreesPerTileAt1m = 0;
representationCount = length(r);
URLbase = 0;
Rep = r;
packetSize = 256; % bytes
numberofPacket = 488; % initial number of packet for one segment

%% Network model
% Model based network throughput generation
networkModel = [20 15 10 5 3 1 3 5 10 15 20 ...
    20 15 10 5 3 1 3 5 10 15 20 ...
    1 3 5 10 15 20 1 3 5 10 15 20 ...
    5 15 5 15 5 15 5]*3e6;
%networkModel = 20e6;
mu = 8*packetSize./networkModel;


%% Client buffer model
cb(1) = clientBufferState(0,0,segmentDuration,[],worldTime);

% initialize
downloadBytes = 0;
downloadTime = 0;

% Get Asset
m1 = manifest;
m2 = submanifest;
A = Asset;
A.manifest = m1;
A.submanifest = m2;

% Select clip to play and speed
AssetClipStartTime = A.manifest.startTime;
AssetClipEndTime = A.manifest.endTime;
AssetClipSpeed = 1;

% state matrix
state = zeros(tileCount*bufGof, 1);

% Read first segment
R(1) = packetSize*numberofPacket*8/segmentDuration; % initial video rate
StartSegment = floor((AssetClipStartTime-A.manifest.startTime)/A.manifest.duration);
%downloadBytes = downloadBytes + A.submanifest.Rep;
ff=1;
% Prefetch
% for ff=1:prefetch
%     for pf = 1:numberofPacket
%         ar = -mu(1)*log(rand); % exprnd(mu(1));
%         worldTime = worldTime + ar;
%     end
%     cb(ff+1) = cb(ff).segmentin(worldTime,R(1));
% end
v = VideoWriter(['SingleAssetTileAndBuffer_Depth' num2str(dd) '.avi']); open(v);
fig = figure(1); set(fig, 'Position', [0 0 800 1000]);
% sfig1 = subplot(5,1,[1 2]); axis(sfig1, [0 ceil(bufGof) 0 tileCount 0 length(r)]); view(sfig1, 10,37.5);
% xlabel('<-- Playback'); ylabel('Tile index'); zlabel('Representation'); title('Window');
%sfig2 = subplot(5,1,[1 2]); hold; axis([0 ceil(numFrame) 0 tileCount 0 length(r)]); view(10,37.5); grid on;
%xlabel('Frames'); ylabel('Tile index'); zlabel('Representation'); title('Buffer Status'); 
sfig3 = subplot(3,1,1); hold; xlabel(sfig3, 'sec'); ylabel(sfig3, 'bps'); title('Estimated Throughput'); grid on;
sfig4 = subplot(3,1,2); hold; xlabel(sfig4, 'sec'); ylabel(sfig4, 'utility'); title('Utility'); grid on;
sfig5 = subplot(3,1,3); hold; xlabel(sfig5, 'sec'); ylabel(sfig5, 'Avg. Representation'); title('Representation'); grid on;


% Loop until done
pb = Playback;
SmoothedChat(1) = 0; f = 1;
userMove = 0; moveSpeed = 1; playBack = 1;
for ss=1:numberofSegment-1
    if mod(ss,10) == 0 && userMove
        tempp = inip;
        inip(1:moveSpeed) = inip(end-moveSpeed+1:end);
        inip(moveSpeed+1:end) = tempp(1:end-moveSpeed);
        p = inip;
        grd = 1;
        for bb=1:bufGof
%             if mod(bb,1)==0, grd = grd+1; end
%             Pmatrix(:,bb) = p/sum(p)/grd;
%             p = conv(p,ones(1,3),'same'); p = p/sum(p);
            Perr = 0.1+0.3*bb/bufGof;
            Pmatrix(:,bb) = p*(1-Perr)+(1-p)*Perr;
            for rr=1:length(r)
                U((bb-1)*tileCount+1:bb*tileCount,rr) = Umatrix(:,rr).*Pmatrix(:,bb);
            end
        end
    end
    bufferTargetDuration = min([W ceil(pb.currentTime+1)]);
    downloadTime = 0;
    %if mod(ss,100) == 0, mu = 0.1*rand(); end
    for pp=1:numberofPacket
        % Packet arrivals - every segment 
        %wB = 0.3;
        %wA = mu(floor(length(mu)*ss/numberofSegment)+1)/gamma(1+1/wB);
        ar = -mu(floor(length(mu)*ss/numberofSegment)+1)*log(rand);
        %ar = wblrnd(wA,wB); % Weibull
        %ar = exprnd(mu(floor(4*ss/num_seg)+1)); % Exponential
        %ar = Ti(pp+1)-Ti(pp); % Time-varying Poisson process
        worldTime(ss) = worldTime(ss) + ar;
        pb.currentTime = worldTime(ss);
        downloadTime = downloadTime + ar; % segment time counter

        % Packet departures - every chunk period
        if floor(worldTime(ss)/segmentDuration) > playOut
            tempPlay = floor(worldTime(ss)/segmentDuration) - playOut;
            for tt=1:tempPlay
                playBack = playBack + 1;
                if cb(ff).o>0, cb(ff+1) = cb(ff).segmentout(worldTime(ss)); 
                else cb(ff+1) = cb(ff);
                end
                tempState = zeros(1,tileCount*bufGof);
                tempState(1:end-tileCount) = state(tileCount+1:end);
                tempState(end-tileCount+1:end) = bufState(:,mod(playBack+bufGof-2,numFrame)+1);
                state = tempState;
                playOut = floor(worldTime(ss)/segmentDuration);
                ff = ff + 1;
            end
        end
        % Buffer occupancy update
    end
    cb(ff+1) = cb(ff).segmentin(worldTime(ss),stateMatrix);
    ff = ff + 1;
    %% Rate adaptation algorithm
    Chat(ss) = R(ss)*segmentDuration/downloadTime;
    SmoothedChat(ss+1) = f*Chat(ss) + (1-f)*SmoothedChat(ss);
    
    switch(4)
        case 1
            % Throughput based algorithm
            %ri = sum(mean(Chat(max([1 (ss-5)]):ss))>r); 
            BW = sum(SmoothedChat(ss+1)>r); 
        case 2
            % Buffer based algorithm
            BW = sum(400000*cb(ff).o>r); 
        case 3
            % Our algorithm with buffer overflow protection
            if cb(ff).o>floor(bufferTargetDuration/segmentDuration)
                BW = sum((Chat(ss)*(cb(ff).o-floor(bufferTargetDuration/segmentDuration)+1))>r*segmentDuration)
            else BW = sum((Chat(ss)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))>r*segmentDuration);
            end
        otherwise 
            % Our algorithm
            %ri = sum((Chat(ss)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))>r*segmentDuration);
            %BW = (SmoothedChat(ss+1)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))/segmentDuration;
            %BW = (SmoothedChat(ss+1)/(floor(bufferTargetDuration)-cb(ff).o))/segmentDuration;
            window = max([floor(bufferTargetDuration)-(cb(ff).o) 1]);
            BW = SmoothedChat(ss+1)/window
    end
    %% Utility Maximization and Make a request
    %state = reshape(stateMatrix(:,1:bufferTargetDuration), [1 tileCount*bufferTargetDuration]);
    %U = reshape(Umatrix, [tileCount*bufferTargetDuration length(r)]);
    %C = reshape(Cmatrix, [tileCount*bufferTargetDuration length(r)]);
    %P = reshape(Pmatrix(:,1:bufferTargetDuration), [1 tileCount*bufferTargetDuration]);
    %window = max([floor((bufferTargetDuration-cb(ff).o)/segmentDuration) 1])
    %state(1:window*tileCount) = TBRS_ARv2(BW, tileCount*window, length(r), U, C, state(1:window*tileCount));
    state = TBRS_ARv2(BW, tileCount*bufGof, length(r), U, C, state);
    %state = TBRS_ARv2(BW, 1*bufGof, length(r), U(1:bufGof,:), C(1:bufGof,:)*tileCount, state);
    stateMatrix = reshape(state(1:tileCount*bufGof), [tileCount bufGof]);
    %stateMatrix = reshape(state(1:1*bufGof), [1 bufGof]);
    bufState(:,mod(playBack-1:playBack-1+bufGof-1,numFrame)+1) = stateMatrix;
    %bufState(1,mod(playBack-1:playBack-1+bufGof-1,numFrame)+1) = stateMatrix;

    % Figure
    %bar3(sfig2, stateMatrix); 
    R(ss+1) = max([1 BW]);
    %R(ss+1) = (SmoothedChat(ss+1)/(floor(bufferTargetDuration/segmentDuration)-cb(ff).o))/segmentDuration;
    numberofPacket = floor(R(ss+1)*segmentDuration/(packetSize*8));
    for tt=1:tileCount
        if stateMatrix(tt,1)>0
            utility(ss,tt)=p(tt).*Umatrix(tt,stateMatrix(tt,1));
        else
            utility(ss,tt)=0;
        end
    end
    averageRep(ss)=mean(stateMatrix(find(inip),1));
    %averageRep(ss)=mean(stateMatrix(1,1));
%     tempBar = ones(tileCount, numFrame); tempBar(:,mod(playBack-1:playBack-1+bufGof-1,numFrame)+1) = 0; 
%     tempBar2 = zeros(tileCount, numFrame); tempBar2(:,mod(playBack-1,numFrame)+1) = 1; 
%     bar3(sfig2, bufState); bar3(sfig2, bufState.*tempBar2,'r'); bar3(sfig2, bufState.*tempBar,'w');  
    %figure(2); bar3(bufState);
    plot(sfig3, worldTime,Chat,'r');  
    plot(sfig4, worldTime,sum(utility,2)/tileCount,'g'); 
    for ll=1:size(cb,2)-1, plot_t(ll) = cb(ll).tt; plot_o(ll) = cb(ll).o; end
    %plot(sfig5, plot_t,plot_o,'b');
    plot(sfig5, worldTime,averageRep,'b');
    drawnow;
    F = getframe(fig);
    writeVideo(v,F);
    worldTime(ss+1) = worldTime(ss);
end
close(v);
figure(3); plot(worldTime(1:end-1),averageRep,'b');
xlabel('sec'); ylabel('Avg. Representation'); title('Representation'); grid on;
% for ll=1:size(cb,2)-1, plot_t(ll) = cb(ll).tt; plot_o(ll) = cb(ll).o; end
% figure(2); 
% subplot(4,1,1); 
% plot(plot_t, plot_o); ylabel('sec'); title('Buffer Occupancy'); 
% text(1,1,['Stall' num2str(cb(end).st)]);
% subplot(4,1,2); 
% plot(segmentDuration*(0:length(Chat)-1),Chat,'r'); ylabel('bps'); title('Estimated Throughput'); 
% text(5,1e7,['Average' num2str(mean(Chat),'%1.2e')]);
% subplot(4,1,3); 
% plot(segmentDuration*(0:length(R)-1),R,'k'); ylabel('bps'); title('Bitrate'); 
% text(5,1e7,['Average' num2str(mean(R),'%1.2e')]);
% subplot(4,1,4);
% plot(segmentDuration*(0:length(utility(:,1))-1),sum(utility,2),'g'); xlabel('sec'); ylabel('utility'); title('Utility');
% text(5,250,['Average' num2str(mean(sum(utility,2)),'%d')]);